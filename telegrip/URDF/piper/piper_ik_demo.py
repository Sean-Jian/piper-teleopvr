import numpy as np
import casadi as ca
import pinocchio as pin
from pinocchio import casadi as cpin
from scipy.spatial.transform import Rotation as R
from piper_sdk import *
import time
"""
用于遥操作piper机械臂算逆解  在核心IK功能的基础上加上了角度跳变检测
"""
class ArmIK:
    def __init__(self, urdf_path: str):
        # 1. 模型加载并锁夹爪
        robot = pin.RobotWrapper.BuildFromURDF(urdf_path)
        self.model = robot.buildReducedRobot(['joint7', 'joint8'],
                                              np.zeros(robot.model.nq)).model
        self.data = self.model.createData()

        # 2. CasADi 模型
        cmodel = cpin.Model(self.model)
        cdata  = cmodel.createData()
        cq, cTf = ca.SX.sym('q', 6), ca.SX.sym('Tf', 4, 4)
        cpin.framesForwardKinematics(cmodel, cdata, cq)

        # 3. 误差函数（log6）
        ee_id = self.model.getFrameId('link6')
        #防止索引越界
        cpin.framesForwardKinematics(cmodel, cdata, cq)
        err = ca.Function('err', [cq, cTf],
                          [ca.vertcat(
                              cpin.log6(cdata.oMf[ee_id].inverse() *
                                        cpin.SE3(cTf)).vector)])

        # 4. 优化问题
        opti = ca.Opti()
        self.var_q    = opti.variable(6)
        self.param_tf = opti.parameter(4, 4)
        # 原版目标：20*poseErr + 0.01*‖q‖²
        opti.minimize(20 * ca.sumsqr(err(self.var_q, self.param_tf)) +
                      0.01 * ca.sumsqr(self.var_q))
        opti.subject_to(opti.bounded(self.model.lowerPositionLimit,
                                     self.var_q,
                                     self.model.upperPositionLimit))
        opti.solver('ipopt', {'ipopt.print_level': 0,
                              'print_time': False,
                              'ipopt.max_iter': 20,
                              'ipopt.tol': 1e-2})
        # 状态变量
        self.opti     = opti
        self.init_q   = np.zeros(6)
        self.last_q   = np.zeros(6)

    def ik(self, A1: np.ndarray):
        """
        4×4 目标位姿齐次矩阵（旋转部分为欧拉角转换的旋转矩阵，平移部分单位为毫米）
        → 6 关节角（度）
        """
        # 创建一个新的矩阵，将平移部分从毫米转换为米
        A1_meters = A1.copy()
        A1_meters[:3, 3] = A1[:3, 3] / 1000.0  # 毫米转米
        
        self.opti.set_value(self.param_tf, A1_meters)
        self.opti.set_initial(self.var_q, self.init_q)
        try:
            self.opti.solve_limited()
            q_sol = self.opti.value(self.var_q)

            # ---- 30° 跳变检测（原版逻辑） ----
            max_diff = np.max(np.abs(q_sol - self.last_q))
            if max_diff > 30.0 * np.pi / 180.0:
                self.init_q = np.zeros(6)   # 重置初值
            else:
                self.init_q = q_sol
            self.last_q = q_sol
            
            # 将弧度转换为度
            q_sol_deg = q_sol * 180.0 / np.pi
            return q_sol_deg
        except Exception:
            return None


# ---------------- demo ----------------/home/robotgym/qijia-teleopvr/telegrip/URDF/piper
if __name__ == '__main__':
    piper = C_PiperInterface_V2("arm_r")
    piper.ConnectPort()
    while( not piper.EnablePiper()):
        time.sleep(0.01)
    print("使能成功")


    pos = [177.0, -2.4, 216.1, -176.1, 16.8, -161.8]
    factor = 1000

    T = np.eye(4)
    T[:3, :3] = R.from_euler('xyz', pos[3:], degrees=True).as_matrix()
    T[:3, 3] = pos[:3]
    ik = ArmIK('/home/robotgym/jxj/qijia-teleopvr/telegrip/URDF/piper/piper_description.urdf')


    start_time = time.time()

    # # #算逆解
    q = ik.ik(T)
    print('关节角 [度]:', q)
    # #关节角度控制
    # joint_0 = round(q[0]*factor)
    # joint_1 = round(q[1]*factor)
    # joint_2 = round(q[2]*factor)
    # joint_3 = round(q[3]*factor)
    # joint_4 = round(q[4]*factor)
    # joint_5 = round(q[5]*factor)
    # piper.ModeCtrl(0x01,0x01,100,0x00)
    # piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)

    #直接末端控制
    # piper.ModeCtrl(0x01,0x00,100,0x00)
    # piper.EndPoseCtrl(round(pos[0]*factor), round(pos[1]*factor), round(pos[2]*factor), round(pos[3]), round(pos[4]), round(pos[5]))

    end_time = time.time()




    print(f"运行时间(ms): {(end_time - start_time) * 1000}")