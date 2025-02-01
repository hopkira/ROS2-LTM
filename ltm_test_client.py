'''
If you want to run this script in a ROS2 environment, 
save it in your package directory (where your services 
are located), then add an entry point to setup.py:

entry_points={
    'console_scripts': [
        ... # your other nodes here
        'test_ltm = test.test_ltm:main',
    ],
},

Finally, you can run the tests with:

ros2 run <your_package> test_ltm

'''
import sys
import unittest
from std_srvs.srv import SetBool as SetBoolReq
from example_interfaces.srv import SetStringList
from ltm_package.msg import StringList # Assuming you have a custom message for this, replace 'ltm_package' with your actual package name
import rclpy
from rclpy.node import Node

class TestLongTermMemoryService(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_set_model(self):
        node = Node('test_ltm')
        client = node.create_client(SetStringList, 'ltm_set_model')
        while not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('Service not available, waiting...')
        req = SetStringList.Request()
        req.data = ['gen', 'test_model'] # replace with your actual data
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        self.assertTrue(future.result().success)
        node.destroy_node()

    def test_query(self):
        # Add your code here to call the 'ltm_query' service and assert some conditions based on the response
        pass

    def test_remember(self):
        # Add your code here to call the 'ltm_remember' service and assert some conditions based on the response
        pass

if __name__ == "__main__":
    unittest.main()