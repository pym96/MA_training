#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/cost_function.h>
#include <ceres/problem.h>
#include <Eigen/Dense>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

namespace aimer::top {

const int FIND_ANGLE_ITERATIONS = 12; // 三分法迭代次数 理想精度 < 1.
const double SIMPLE_TOP_TRACK_AREA_RATIO = 2.;
const double DETECTOR_ERROR_PIXEL_BY_SLOPE = 2.;

double get_pts_cost(
    const std::vector<cv::point2f>& cv_refs,
    const std::vector<cv::point2f>& cv_pts,
    const double& inclined
) {
    std::size_t size = cv_refs.size();
    std::vector<eigen::vector2d> refs;
    std::vector<eigen::vector2d> pts;
    for (std::size_t i = 0u; i < size; ++i) {
        refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
        pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
    }
    double cost = 0.;
    for (std::size_t i = 0u; i < size; ++i) {
        std::size_t p = (i + 1u) % size;
        // i - p 构成线段。过程：先移动起点，再补长度，再旋转
        eigen::vector2d ref_d = refs[p] - refs[i]; // 标准
        Eigen::Vector2d pt_d = pts[p] - pts[i];
        // 长度差代价 + 起点差代价(1 / 2)（0 度左右应该抛弃)
        double pixel_dis = // dis 是指方差平面内到原点的距离
            (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm())
             + std::fabs(ref_d.norm() - pt_d.norm()))
            / ref_d.norm();
        double angular_dis = ref_d.norm() * aimer::math::get_abs_angle(ref_d, pt_d) / ref_d.norm();
        // 平方可能是为了配合 sin 和 cos
        // 弧度差代价（0 度左右占比应该大）
        double cost_i = math::sq(pixel_dis * std::sin(inclined))
            + math::sq(angular_dis * std::cos(inclined)) * top::DETECTOR_ERROR_PIXEL_BY_SLOPE;
        // 重投影像素误差越大，越相信斜率
        cost += std::sqrt(cost_i);
    }
    return cost;
}

std::vector<Eigen::Vector3d> radial_armor_corners(
    const Eigen::Vector3d& pos,
    const aimer::ArmorType& type,
    const double& pitch,
    const double& z_to_v,
    aimer::CoordConverter* const converter
) {
    const std::vector<cv::Point3d>& pw =
        type == aimer::ArmorType::BIG ? aimer::PW_BIG : aimer::PW_SMALL;
    Eigen::Vector2d radius_norm = aimer::math::rotate(converter->get_camera_z_i2(), z_to_v);
    Eigen::Vector3d x_norm;
    x_norm << aimer::math::rotate(radius_norm, M_PI / 2.), 0.;
    Eigen::Vector3d w_z_norm { 0., 0., 1. }; // 不要把变量写的像函数
    Eigen::Vector3d y_norm;
    y_norm << -radius_norm * std::sin(pitch), 0.;
    y_norm += w_z_norm * std::cos(pitch);
    std::vector<Eigen::Vector3d> corners;
    for (int i = 0; i < 4; ++i) {
        corners.push_back(pos + x_norm * pw[i].x + y_norm * pw[i].y);
    }
    return corners;
}

std::vector<cv::Point2f> radial_armor_pts(
    const Eigen::Vector3d& pos,
    const aimer::ArmorType& type,
    const double& pitch,
    const double& z_to_v,
    aimer::CoordConverter* const converter
) {
    std::vector<Eigen::Vector3d> corners =
        top::radial_armor_corners(pos, type, pitch, z_to_v, converter);
    std::vector<cv::Point2f> pts;
    for (int i = 0; i < 4; ++i) {
        pts.push_back(converter->pi_to_pu(corners[i]));
    }
    return pts;
}

std::vector<std::vector<cv::Point2f>> radial_double_pts(
    const std::vector<aimer::ArmorData>& armors,
    const double& z_to_l,
    aimer::CoordConverter* const converter
) {
    std::vector<std::vector<cv::Point2f>> res;
    res.push_back(top::radial_armor_pts(
        armors[0].info.pos,
        armors[0].info.sample.type,
        armors[0].info.orientation_pitch_under_rule,
        z_to_l,
        converter
    ));
    res.push_back(top::radial_armor_pts(
        armors[1].info.pos,
        armors[1].info.sample.type,
        armors[0].info.orientation_pitch_under_rule,
        z_to_l + M_PI / 2.,
        converter
    ));
    return res;
}

double SingleCost::operator()(const double& x) {
    // value: z_to_l
    std::vector<cv::Point2f> pts = top::radial_armor_pts(
        this->data.d.info.pos,
        this->data.d.info.sample.type,
        this->data.d.info.orientation_pitch_under_rule,
        x,
        this->data.converter
    );
    return top::get_pts_cost(
        pts,
        std::vector<cv::Point2f> {
            this->data.d.info.pus[0],
            this->data.d.info.pus[1],
            this->data.d.info.pus[2],
            this->data.d.info.pus[3],
        },
        this->data.z_to_v_exp
    );
}

double DoubleCost::operator()(const double& x) {
    double z_to_r_exp = this->data.z_to_l_exp + M_PI / 2.;
    std::vector<std::vector<cv::Point2f>> ml_pts =
        top::radial_double_pts(this->data.armors, x, this->data.converter);
    return top::get_pts_cost(
               ml_pts[0],
               std::vector<cv::Point2f> {
                   this->data.armors[0].info.pus[0],
                   this->data.armors[0].info.pus[1],
                   this->data.armors[0].info.pus[2],
                   this->data.armors[0].info.pus[3],
               },
               this->data.z_to_l_exp
           )
        + top::get_pts_cost(
               ml_pts[1],
               std::vector<cv::Point2f> {
                   this->data.armors[1].info.pus[0],
                   this->data.armors[1].info.pus[1],
                   this->data.armors[1].info.pus[2],
                   this->data.armors[1].info.pus[3],
               },
               z_to_r_exp
        );
}

double fit_single_z_to_v(
    const aimer::ArmorData& armor,
    const double& z_to_v_exp,
    const double& z_to_v_min,
    const double& z_to_v_max,
    aimer::CoordConverter* const converter
) {
    top::SingleCost single_cost = top::SingleCost(top::SingleData(armor, z_to_v_exp, converter));
    aimer::math::Trisection solver;
    std::pair<double, double> res =
        solver.find(z_to_v_min, z_to_v_max, single_cost, top::FIND_ANGLE_ITERATIONS);
    return aimer::math::reduced_angle(res.first);
    // double res = (z_to_v_min + z_to_v_max) / 2.;
    // ceres::Problem problem;
    // ceres::CostFunction* cost_function{
    //     new ceres::AutoDiffCostFunction<top::SingleCostFunctor, 1, 1>(
    //         new top::SingleCostFunctor(
    //             top::SingleData(armor, z_to_v_exp, converter)))};
    // problem.AddResidualBlock(cost_function, nullptr, &res);
    // ceres::Solver::Options options;
    // options.linear_solver_type = ceres::DENSE_QR;
    // options.minimizer_progress_to_stdout = true;
    // ceres::Solver::Summary summary;
    // ceres::Solve(options, &problem, &summary);
    // return aimer::math::reduced_angle(res);
}

// 拟合得到角度。数据，限制，数据库
double fit_double_z_to_l(
    const std::vector<aimer::ArmorData>& armors,
    const double& z_to_l_exp,
    const double& z_to_l_min,
    const double& z_to_l_max,
    aimer::CoordConverter* const converter
) {
    top::DoubleCost double_cost { top::DoubleCost(top::DoubleData(armors, z_to_l_exp, converter)) };
    aimer::math::Trisection solver;
    std::pair<double, double> res =
        solver.find(z_to_l_min, z_to_l_max, double_cost, top::FIND_ANGLE_ITERATIONS);
    return aimer::math::reduced_angle(res.first);
}

// 不要为了规范而随便改旧的风格
// 1. 不知道旧的风格会不会被重新启用，主要看可读性
// 2. 提供旧风格与新风格的比较

aimer::AimInfo get_top_limit_aim(
    const top::CenterFilter& center_filter,
    const std::vector<top::TopArmor>& prediction_results,
    const std::vector<top::TopArmor>& hit_results,
    const double& top_w,
    aimer::CoordConverter* const converter,
    aimer::EnemyState* const state
) {
    /// 获取瞄准点思路：
    /// 非常复杂...
    struct AimArmor {
        aimer::AimInfo emerging_aim;
        top::TopArmor armor;
    };
    auto get_direction = [](const double& top_w) -> top::TopDirection {
        return top_w > 0. ? top::TopDirection::CCW : top::TopDirection::CW;
    };

    auto get_directs = [](const std::vector<top::TopArmor>& results) -> std::vector<top::TopArmor> {
        std::vector<top::TopArmor> directs;
        for (const auto& res: results) {
            if (std::fabs(res.zn_to_v) < aimer::math::deg_to_rad(
                    base::get_param<double>("auto-aim.top-model.aim.max-orientation-angle")
                ))
            {
                directs.push_back(res);
            }
        }
        return directs;
    };

    auto choose_direct_aim = [&converter, &center_filter, &top_w, &get_direction](
                                 const std::vector<top::TopArmor>& directs
                             ) -> AimArmor {
        top::TopArmor direct = directs[0];
        for (const auto& d: directs) {
            // aim_swing_cost 利用的是与图像绑定的陀螺仪（枪口指向）信息选择打击板
            if (converter->aim_swing_cost(converter->target_pos_to_aim_ypd(d.pos))
                < converter->aim_swing_cost(converter->target_pos_to_aim_ypd(direct.pos)))
            {
                direct = d;
            }
        }
        aimer::AimInfo aim = [&]() {
            // 此处求解空气阻力函数复用
            aimer::ShootParam shoot_param = converter->target_pos_to_shoot_param(direct.pos);
            aimer::math::YpdCoord ypd =
                converter->aim_xyz_i_to_aim_ypd(shoot_param.aim_xyz_i_barrel);
            // 计算 yaw_v
            // 计算方法为“预测击中时敌人自角度”下根据“此刻角速度”推算 xy
            // 速度，然后 ypd 速度
            Eigen::Vector2d zn_norm2 = aimer::math::rotate(converter->get_camera_z_i2(), M_PI);
            // v_norm 是装甲板向量
            Eigen::Vector2d v_norm2 = aimer::math::rotate(zn_norm2, direct.zn_to_v);
            // xy 方向上的速度
            Eigen::Vector2d xy_v =
                aimer::math::rotate(
                    v_norm2,
                    get_direction(top_w) == top::TopDirection::CCW ? +M_PI / 2. // 垂直量
                                                                   : -M_PI / 2.
                )
                * direct.length * std::fabs(top_w);
            Eigen::Vector3d xyz_v = { xy_v(0, 0), xy_v(1, 0), 0. };
            aimer::math::YpdCoord ypd_v = converter->filter_to_aim_ypd_v(center_filter)
                + converter->get_camera_ypd_v(direct.pos, xyz_v);
            return aimer::AimInfo(ypd, ypd_v, shoot_param, ::ShootMode::IDLE);
        }();
        return AimArmor { aim, direct };
    };

    // results 应该包含时间
    auto get_indirect_aim = [&converter, &state, &center_filter, &top_w, &get_direction](
                                const std::vector<top::TopArmor>& results
                            ) -> AimArmor {
        double zn_to_lim = get_direction(top_w) == top::TopDirection::CCW
            ? -base::get_param<double>("auto-aim.top-model.aim.max-orientation-angle") / 180. * M_PI
            : +base::get_param<double>("auto-aim.top-model.aim.max-orientation-angle") / 180.
                * M_PI;
        // 所有装甲板距 limit 角的角度中最小的
        double closest_to_lim = aimer::INF;
        top::TopArmor indirect = results[0];
        for (const auto& armor: results) {
            // 跟踪限制最大角
            // 寻找即将出现的板子
            // 在新等待位置时允许超额打击（上一块板子）的角度，
            // 注意对于刚刚过去的板子，如果没有新目标就会傻傻等待并
            // 射击，但严格要求不跟踪（如英雄 0
            // 度)如果有新目标就会进入上方的 direct
            // 但当限制角很小的时候有可能打击这个刚刚过去的板子，此时
            // closest 是负值
            double leaving_angle =
                (state->get_sample_armor_ref().width / 2.
                 * base::get_param<double>("auto-aim.top-model.aim.max-out-error"))
                / armor.length;
            double armor_to_lim =
                aimer::math::reduced_angle(
                    (get_direction(top_w) == TopDirection::CCW ? zn_to_lim - armor.zn_to_v
                                                               : armor.zn_to_v - zn_to_lim)
                    - M_PI + leaving_angle
                )
                + M_PI - leaving_angle; // -leave ~ 2pi - leave
            if (armor_to_lim < closest_to_lim) {
                indirect = armor; // indirect
                closest_to_lim = armor_to_lim; // 允许负值出现，但是并不会跟踪
            }
        }
        // 匀速时理想 lim_center 固定
        // lim 度（例如 0）时的中心（欲瞄准
        // 用这个中心延长的位置）
        Eigen::Vector3d center_lim =
            center_filter.predict_pos(results[0].t + closest_to_lim / std::fabs(top_w));
        // what if top_w == 0?
        // 特例：当 lim = 0 且装甲板朝向已经转过去 5 度时，瞄准的依然是
        // 这块板在 5 度之前（正对）所在位置，此时 closest 的作用是
        // 防止枪口转到下一个装甲板正对时的位置
        Eigen::Vector2d center_lim2 = { center_lim(0, 0), center_lim(1, 0) };
        Eigen::Vector2d lim_norm2 =
            aimer::math::rotate(converter->get_camera_z_i2(), M_PI + zn_to_lim);
        Eigen::Vector2d emerging_pos2 = center_lim2 + lim_norm2 * indirect.length;
        Eigen::Vector3d emerging_pos = { emerging_pos2(0, 0),
                                         emerging_pos2(1, 0),
                                         center_lim(2, 0) + indirect.z_plus };
        aimer::AimInfo aim = [&]() {
            aimer::ShootParam shoot_param = converter->target_pos_to_shoot_param(emerging_pos);
            aimer::math::YpdCoord ypd =
                converter->aim_xyz_i_to_aim_ypd(shoot_param.aim_xyz_i_barrel);
            aimer::math::YpdCoord ypd_v = converter->filter_to_aim_ypd_v(center_filter);
            return aimer::AimInfo(ypd, ypd_v, shoot_param, ::ShootMode::IDLE);
        }();
        // 之前打击逻辑直接统一 在这里 indirect.pos
        return AimArmor { aim, indirect }; // 是根据 lim 延伸出来的
    };

    auto get_aim_angle = [&get_directs, &choose_direct_aim, &get_indirect_aim](
                             const std::vector<top::TopArmor>& results
                         ) -> AimArmor {
        std::vector<top::TopArmor> directs = get_directs(results);
        return !directs.empty() ? choose_direct_aim(directs) : get_indirect_aim(results);
    };

    aimer::AimInfo tracking_aim = get_aim_angle(prediction_results).emerging_aim;
    { // 以下判断 shoot
        // 常规 armor_model 的判断发弹为：当前是否足够收敛（理想 yaw 始终为 0）
        // 假设子弹为光速，那只要跟紧就根本不应该考虑发弹延迟，否则预估是错误的
        // 但实际上，就算当前不收敛，shoot cmd 延迟 = 3s 之
        // 后可能早已收敛，没办法预测自己在这么大延迟后的表现

        // 反陀螺则判断条件更少，因为更无法预测自己在极大 shoot cmd 延迟后是否收敛
        // 此时我们只能认为它保持收敛。判断发弹的方法更为基本，仅是该角度下被选定
        // 的板是否可能被打击
        // 但有一段时间（旋转到待击打点期间）不可能收敛

        // 若发射延迟达到装甲板切换间隔的一半，需要严肃考虑是否给发射指令
        // 此时给发射，击打位置在哪里？
        // img 的延迟也需要考虑。我们计算方法是时间轴上的击打时间点
        // 1. hit: 如果 hit
        // 位置需要旋转角太大，那就不打（主要是对于切换等待期间的限制）
        // 常规预测并无此判断，当发弹延迟巨大时，这一判断反而是累赘
        // 范围装甲板的打击限制）
        // 无法根据对 hit 收敛程度判断是否发射，hit 没有收敛之说 可否通过对
        AimArmor hit_aim = get_aim_angle(hit_results);
        bool for_a_hit = // 1. 电机有没有跟上
            converter->aim_error_exceeded(
                tracking_aim.ypd,
                state->get_sample_armor_ref(),
                state->get_aim_error(),
                hit_aim.armor.zn_to_v,
                state->get_armor_pitch()
            )
                // 2. 若打中，但将要打中的位置和当前枪口指向差的老远，就不打
                // 这玩意能删么
                // 不能删啊，跟随一块装甲板的最后一段时间是不能发发弹指令的
                // 改成 tracking 和 hitting 的差会不会更好？
                // 好像不行捏
                // 还是用来防止
                || converter->aim_error_exceeded(
                    hit_aim.emerging_aim.ypd,
                    state->get_sample_armor_ref(),
                    base::get_param<double>("auto-aim.top-model.aim.max-swing-error"),
                    hit_aim.armor.zn_to_v,
                    state->get_armor_pitch()
                )
                ||
                // 3. 这里函数是双 aim 比较的重载
                // 若打中，打中的 emerging pos 和实际装甲板位置的差，就是 out-error
                converter->aim_error_exceeded(
                    hit_aim.emerging_aim.ypd,
                    converter->target_pos_to_aim_ypd(hit_aim.armor.pos),
                    state->get_sample_armor_ref(),
                    base::get_param<double>("auto-aim.top-model.aim.max-out-error"),
                    hit_aim.armor.zn_to_v,
                    state->get_armor_pitch()
                )
            ? false
            : true;
        tracking_aim.shoot = for_a_hit ? ::ShootMode::SHOOT_NOW : ::ShootMode::TRACKING;
    }
    return tracking_aim;
}

}
