import json
import numpy as np
import matplotlib.pyplot as plt


def analyze_simulation(source, plot=False, return_metrics=True, verbose=True):
    """
    【增强版】支持两种使用模式：
    1. 从文件读取（str） → 兼容你原来的 analyze_simulation("simulation_log_xxx.txt")
    2. 从内存 history_data 直接读取（dict） → 自动 PID 整定时实时计算 cost（无需存盘/读盘，速度更快）

    参数说明：
        source:         str（文件路径） 或 dict（history_data）
        plot:           是否弹出 Matplotlib 图表（自动整定时建议 False，避免弹窗卡死）
        return_metrics: 是否返回 metrics 字典（供 cost 计算使用）
        verbose:        是否打印评估报告（整定时可设 False 更安静）
    """
    # ====================== 1. 加载数据（两种模式） ======================
    if isinstance(source, str):
        try:
            with open(source, 'r', encoding='utf-8') as f:
                data = json.load(f)
            if verbose:
                print(f"✅ 从文件加载: {source}")
        except FileNotFoundError:
            print(f"❌ 找不到文件: {source}")
            return None
    elif isinstance(source, dict):
        data = source
        if verbose:
            print("✅ 从内存 history_data 直接读取（自动整定模式）")
    else:
        print("❌ source 必须是文件路径(str) 或 history_data(dict)")
        return None

    # ====================== 2. 数据转换 & 计算指标 ======================
    traj = np.array(data["trajectory"])
    errors = np.array(data["error"])  # [dist_err, ang_err]
    controls = np.array(data["control"])  # [v, w]

    dist_err = errors[:, 0]
    ang_err = errors[:, 1]

    # 你的原始指标 + 更适合 PID 自动整定的高级指标
    metrics = {
        "RMSE (位置误差)": np.sqrt(np.mean(dist_err ** 2)),
        "Max Error (最大偏差)": np.max(dist_err),
        "Mean Absolute Error (平均误差)": np.mean(np.abs(dist_err)),
        "Control Smoothness (速度平滑度)": np.mean(np.abs(np.diff(controls[:, 0]))),
        "Total Distance (行驶总距离)": np.sum(np.sqrt(np.sum(np.diff(traj, axis=0) ** 2, axis=1))),

        # 新增：PID 优化常用 cost 指标（直接拿来当 cost 非常合适）
        "ISE (平方误差积分)": np.sum(dist_err ** 2),  # Integral Square Error
        "IAE (绝对误差积分)": np.sum(np.abs(dist_err)),  # Integral Absolute Error
        "ITAE (时间加权误差积分)": np.sum(np.arange(len(dist_err)) * np.abs(dist_err)),
        "Final Error": float(dist_err[-1]) if len(dist_err) > 0 else 0.0,
    }

    # ====================== 3. 打印报告 ======================
    if verbose:
        print("\n--- 控制器性能评估报告 ---")
        for key, value in metrics.items():
            print(f"{key}: {value:.4f}")

    # ====================== 4. 可视化（仅 plot=True 时执行） ======================
    if plot:
        plt.figure(figsize=(12, 8))

        plt.subplot(2, 2, 1)
        plt.plot(traj[:, 0], traj[:, 1], 'b-', label='Actual Path')
        plt.scatter(traj[0, 0], traj[0, 1], c='g', marker='o', label='Start')
        plt.scatter(traj[-1, 0], traj[-1, 1], c='r', marker='x', label='End')
        plt.title("Robot Trajectory")
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.legend()
        plt.grid(True)

        plt.subplot(2, 2, 2)
        plt.plot(dist_err, 'r-', label='Distance Error')
        plt.title("Tracking Error over Time")
        plt.xlabel("Frames")
        plt.ylabel("Error (m)")
        plt.grid(True)

        plt.subplot(2, 2, 3)
        plt.plot(controls[:, 0], label='Linear Vel (v)')
        plt.plot(controls[:, 1], label='Angular Vel (w)')
        plt.title("Control Commands")
        plt.xlabel("Frames")
        plt.ylabel("Velocity")
        plt.legend()
        plt.grid(True)

        plt.subplot(2, 2, 4)
        plt.plot(np.degrees(ang_err), 'g-', label='Angle Error (deg)')
        plt.title("Heading Error")
        plt.xlabel("Frames")
        plt.ylabel("Degrees")
        plt.grid(True)

        plt.tight_layout()
        plt.show()

    return metrics if return_metrics else None


# ====================== 示例：两种模式都支持 ======================
if __name__ == "__main__":
    # 模式1：文件模式（完全兼容你原来的调用）
    analyze_simulation("simulation_log_default.txt", plot=True)

    # 模式2：内存模式（自动整定时推荐）
    # dummy_history_data = { "trajectory": [...], "error": [...], "control": [...] }
    # metrics = analyze_simulation(dummy_history_data, plot=False)
    # print("cost 可以直接用 metrics['ISE (平方误差积分)'] 或 metrics['RMSE (位置误差)']")
    pass