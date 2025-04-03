import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import numpy as np
from sklearn import svm
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report
from joblib import dump

class SVMTrainerNode(Node):
    def __init__(self):
        super().__init__('semg_svm_trainer')

        # Feature vector subscriber
        self.subscriber_ = self.create_subscription(
            Float32MultiArray,
            'semg_features',
            self.feature_callback,
            10
        )

        # Label input subscriber (for user to input data labels)
        self.label_sub_ = self.create_subscription(
            String,
            'gesture_label',
            self.label_callback,
            10
        )

        self.feature_buffer = []
        self.label_buffer = []
        self.current_label = None
        self.model = None
        self.get_logger().info('SVM Trainer Node initialized. Publish features to /semg_features and labels to /gesture_label.')

    def feature_callback(self, msg):
        if self.current_label is not None:
            self.feature_buffer.append(msg.data)
            self.label_buffer.append(self.current_label)
            self.get_logger().info(f'Feature vector received and labeled as: {self.current_label}')

    def label_callback(self, msg):
        self.current_label = msg.data
        self.get_logger().info(f'Current training label set to: {self.current_label}')

    def train_model(self):
        if len(self.feature_buffer) < 10:
            self.get_logger().warn('Not enough data to train.')
            return

        X = np.array(self.feature_buffer)
        y = np.array(self.label_buffer)

        self.get_logger().info(f'Training SVM on {len(X)} samples...')

        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2)
        self.model = svm.SVC(kernel='linear')
        self.model.fit(X_train, y_train)

        y_pred = self.model.predict(X_test)
        report = classification_report(y_test, y_pred)
        self.get_logger().info(f"\n{report}")

        dump(self.model, 'svm_gesture_model.pkl')
        self.get_logger().info('SVM model trained and saved to svm_gesture_model.pkl')

def main(args=None):
    rclpy.init(args=args)
    node = SVMTrainerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted. Training model before shutdown...')
        node.train_model()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
