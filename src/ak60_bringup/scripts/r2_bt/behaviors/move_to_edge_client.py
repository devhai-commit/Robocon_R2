import py_trees
import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

# Thay thế bằng tên package chứa interface của bạn
from ak60_bringup.action import MoveToEdge

class MoveToEdgeClient(py_trees.behaviour.Behaviour):
    """
    Behavior Tree Leaf Node đóng vai trò là Action Client gọi tới MoveToEdge Action Server.
    """
    def __init__(self, name, ros_node, speed=0.1, change_threshold=0.04):
        # Khởi tạo node với tên Behavior
        super().__init__(name)
        
        self.ros_node = ros_node
        self.speed = speed
        self.change_threshold = change_threshold
        
        # Biến giữ kết nối Action Client
        self.action_client = None
        
        # Các biến trạng thái của luồng ROS 2 Action
        self.goal_future = None
        self.result_future = None
        self.goal_handle = None

    def setup(self, **kwargs):
        """
        Được gọi 1 lần duy nhất khi khởi tạo Cây hành vi.
        Dùng để thiết lập kết nối tới ROS 2 Server.
        """
        self.logger.debug(f"[{self.name}] Khởi tạo kết nối tới Action Server 'move_to_edge'...")
        self.action_client = ActionClient(self.ros_node, MoveToEdge, 'move_to_edge')
        
        # Chờ server khả dụng
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.logger.error(f"[{self.name}] Không tìm thấy Action Server!")
            raise RuntimeError("MoveToEdge Action Server is not available.")
            
        self.logger.debug(f"[{self.name}] Đã kết nối thành công với Action Server.")

    def initialise(self):
        """
        Được gọi mỗi khi Node này bắt đầu chuyển sang trạng thái RUNNING.
        Dùng để đóng gói và gửi Goal đi.
        """
        self.logger.debug(f"[{self.name}] Bắt đầu gửi lệnh tiến tìm mép bậc...")
        
        goal_msg = MoveToEdge.Goal()
        goal_msg.speed = self.speed
        goal_msg.change_threshold = self.change_threshold
        
        # Gửi goal bất đồng bộ và gán callback để đọc feedback
        self.goal_future = self.action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        
        # Đặt lại các biến kết quả
        self.result_future = None
        self.goal_handle = None

    def feedback_callback(self, feedback_msg):
        """ Xử lý dữ liệu phản hồi liên tục từ Server """
        dist = feedback_msg.feedback.current_distance
        self.logger.debug(f"[{self.name}] Feedback: Khoảng cách laser hiện tại = {dist:.3f}m")

    def update(self):
        """
        Được gọi liên tục trong mỗi nhịp (tick) của Cây hành vi.
        Nhiệm vụ: Đánh giá tiến độ của Action mà không chặn luồng chính.
        """
        # Trạng thái 1: Đang chờ Server phản hồi xem có chấp nhận Goal không
        if self.goal_future and not self.goal_future.done():
            return py_trees.common.Status.RUNNING

        # Trạng thái 2: Server đã nhận Goal, kiểm tra chấp nhận hay từ chối
        if self.goal_future and self.goal_future.done() and not self.result_future:
            self.goal_handle = self.goal_future.result()
            
            if not self.goal_handle.accepted:
                self.logger.warning(f"[{self.name}] Server đã TỪ CHỐI Goal.")
                return py_trees.common.Status.FAILURE
            
            # Nếu chấp nhận, bắt đầu lắng nghe kết quả thực thi
            self.logger.debug(f"[{self.name}] Goal được chấp nhận. Đang thực thi...")
            self.result_future = self.goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING

        # Trạng thái 3: Đang thực thi (di chuyển), chờ kết quả cuối cùng
        if self.result_future and not self.result_future.done():
            return py_trees.common.Status.RUNNING

        # Trạng thái 4: Server đã trả về kết quả
        if self.result_future and self.result_future.done():
            result = self.result_future.result()
            status = result.status
            
            if status == GoalStatus.STATUS_SUCCEEDED:
                final_dist = result.result.final_distance
                self.logger.info(f"[{self.name}] THÀNH CÔNG! Đã dừng tại mép (Khoảng cách chốt: {final_dist:.3f}m).")
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.error(f"[{self.name}] THẤT BẠI. Status Code: {status}")
                return py_trees.common.Status.FAILURE

        # Fallback an toàn
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """
        Được gọi khi Node kết thúc (SUCCESS/FAILURE) hoặc bị ngắt ngang (INVALID).
        """
        # Nếu nhánh này bị ngắt (Preempted) bởi một Selector cấp cao hơn
        # (Ví dụ: cảm biến Lidar phát hiện có người cắt ngang mũi robot)
        if new_status == py_trees.common.Status.INVALID:
            self.logger.debug(f"[{self.name}] Bị ngắt ngang (Preempted)! Gửi lệnh Cancel tới Server...")
            
            if self.goal_handle is not None and not self.result_future.done():
                self.goal_handle.cancel_goal_async()
        
        # Reset các biến trạng thái chuẩn bị cho lần tick kế tiếp
        self.goal_future = None
        self.result_future = None
        self.goal_handle = None


# Ví dụ sử dụng trong một Behavior Tree
if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node('move_to_edge_client_node')
    
    move_to_edge_behavior = MoveToEdgeClient("MoveToEdge", node, speed=0.1, change_threshold=0.04)
    
    # Thiết lập và chạy Behavior Tree (ví dụ đơn giản)
    root = py_trees.composites.Sequence("Root")
    root.add_child(move_to_edge_behavior)
    
    tree = py_trees.trees.BehaviourTree(root)
    
    try:
        tree.setup(timeout=15)
        while rclpy.ok():
            tree.tick()
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()    