# ====================== pso_pid_optimizer.py ======================
# 【这是独立的 PSO 类文件，请单独保存为 pso_pid_optimizer.py】
import numpy as np
import matplotlib.pyplot as plt

class PSO:
    """
    Particle Swarm Optimization (PSO) 类
    - 完全独立，可复用
    - 新增：自动记录每次迭代的最优 cost，用于最终绘制收敛曲线
    """
    def __init__(self, n_particles=15, max_iter=25,
                 w=0.73, c1=1.5, c2=1.6, verbose=True):
        self.n_particles = n_particles
        self.max_iter = max_iter
        self.w = w
        self.c1 = c1
        self.c2 = c2
        self.verbose = verbose
        self.best_cost_history = []   # ← 新增：用于绘制收敛曲线

    def optimize(self, objective, bounds):
        dim = len(bounds)
        lb = np.array([b[0] for b in bounds])
        ub = np.array([b[1] for b in bounds])

        particles = np.random.uniform(lb, ub, (self.n_particles, dim))
        velocities = np.zeros((self.n_particles, dim))
        pbest = particles.copy()
        pbest_cost = np.full(self.n_particles, np.inf)
        gbest = None
        gbest_cost = np.inf

        self.best_cost_history = []   # 清空历史

        print(f"🚀 PSO 开始优化 | 粒子数={self.n_particles} | 最多迭代 {self.max_iter} 次")

        for it in range(self.max_iter):
            for i in range(self.n_particles):
                cost = objective(particles[i])

                if cost < pbest_cost[i]:
                    pbest[i] = particles[i].copy()
                    pbest_cost[i] = cost
                if cost < gbest_cost:
                    gbest = particles[i].copy()
                    gbest_cost = cost

            # 更新速度和位置
            for i in range(self.n_particles):
                r1 = np.random.rand(dim)
                r2 = np.random.rand(dim)
                velocities[i] = (self.w * velocities[i] +
                                 self.c1 * r1 * (pbest[i] - particles[i]) +
                                 self.c2 * r2 * (gbest - particles[i]))
                particles[i] += velocities[i]
                particles[i] = np.clip(particles[i], lb, ub)

            self.best_cost_history.append(gbest_cost)   # ← 记录每次迭代的最优 cost

            if self.verbose and (it % 5 == 0 or it == self.max_iter - 1):
                print(f"   Iter {it+1:3d}/{self.max_iter} | 当前全局最优 cost = {gbest_cost:.4f}")

        print("🎉 PSO 优化完成！")
        return gbest, gbest_cost

    def plot_convergence(self, save_path="./pso_convergence.png"):
        """PSO 结束后绘制收敛曲线（可保存为图片）"""
        plt.figure(figsize=(10, 6))
        plt.plot(self.best_cost_history, 'b-', linewidth=2, label='Best Cost')
        plt.title("PSO Optimization Convergence Curve\n(Best Cost per Iteration)")
        plt.xlabel("Iteration")
        plt.ylabel("Cost (ISE)")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(save_path)
        plt.show()
        print(f"📊 收敛曲线已保存至: {save_path}")