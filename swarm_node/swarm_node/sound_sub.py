import os
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"  # Force CPU

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int32
import tensorflow_hub as hub
import tensorflow as tf
import pandas as pd
import numpy as np
from scipy.fft import fft, ifft


class SoundScore(Node):
    def __init__(self):
        super().__init__('audio_node')

        # ROS publisher (optional, you can use to publish predictions)
        self.publisher = self.create_publisher(Int32, 'speech_probability', 10)

        # Subscribe to published audio chunks
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'human_voice',
            self.score_func,
            10
        )

        # Parameters for noise reduction
        self.CHUNK = 1024  # samples per buffer
        self.NOISE_FRAMES = 5  # initial frames to estimate noise

        # Load YAMNet model once
        self.get_logger().info("Loading YAMNet model...")
        self.yamnet_model = hub.load("https://tfhub.dev/google/yamnet/1")
        self.get_logger().info("✅ YAMNet loaded")

        # Load class map once
        class_map_path = tf.keras.utils.get_file(
            'yamnet_class_map.csv',
            'https://raw.githubusercontent.com/tensorflow/models/master/research/audioset/yamnet/yamnet_class_map.csv'
        )
        self.class_map = pd.read_csv(class_map_path)

    def spectral_subtraction(self, signal_frame, noise_profile, alpha=2.0):
        """
        Apply spectral subtraction to one audio frame.
        """
        signal_spec = fft(signal_frame)
        signal_mag = np.abs(signal_spec)
        signal_phase = np.angle(signal_spec)
        signal_power = signal_mag**2

        # Subtract noise power
        clean_power = signal_power - alpha * noise_profile
        clean_power[clean_power < 0] = 0  # half-wave rectification

        clean_mag = np.sqrt(clean_power)
        clean_spec = clean_mag * np.exp(1j * signal_phase)
        clean_frame = ifft(clean_spec).real
        return clean_frame.astype(signal_frame.dtype)

    def live_noise_reduction_from_array(self, data_array):
        """
        Process a numpy array of audio samples as if it were live audio.
        """
        total_data_points = len(data_array)
        processed_frames = []

        if total_data_points < (self.NOISE_FRAMES + 1) * self.CHUNK:
            self.get_logger().error("Not enough data to estimate noise and process.")
            return np.array([], dtype=np.float32)

        # 1. Estimate noise profile from first NOISE_FRAMES
        noise_power_profile = np.zeros(self.CHUNK, dtype=np.float64)
        for i in range(self.NOISE_FRAMES):
            start = i * self.CHUNK
            end = start + self.CHUNK
            noise_frame = data_array[start:end].astype(np.float64)
            noise_power_profile += np.abs(fft(noise_frame))**2
        noise_power_profile /= self.NOISE_FRAMES

        # 2. Process remaining frames
        current_index = self.NOISE_FRAMES * self.CHUNK
        while current_index + self.CHUNK <= total_data_points:
            audio_frame = data_array[current_index:current_index + self.CHUNK].astype(np.float64)
            processed_frame = self.spectral_subtraction(audio_frame, noise_power_profile)
            processed_frames.append(processed_frame)
            current_index += self.CHUNK

        # Flatten to 1D waveform for YAMNet
        return np.concatenate(processed_frames)

    def score_func(self, msg):
        """
        Callback for ROS subscription. Takes published audio array, applies noise reduction,
        and feeds into YAMNet.
        """
        # Convert ROS Int32MultiArray to numpy float32
        waveform = np.array(msg.data, dtype=np.float32)
        # Normalize if your publisher sends int32 audio
        waveform /= np.iinfo(np.int32).max

        # Apply noise reduction
        waveform_rectified = self.live_noise_reduction_from_array(waveform)

        if waveform_rectified.size == 0:
            self.get_logger().warn("Skipping empty processed waveform")
            return

        # Run YAMNet
        scores, embeddings, spectrogram = self.yamnet_model(waveform_rectified)
        scores = scores.numpy()

        # Average scores across time
        mean_scores = np.mean(scores, axis=0)

        # Get "Speech" class probability
        speech_index = self.class_map[self.class_map['display_name'] == 'Speech'].index[0]
        speech_prob = mean_scores[speech_index]

        # Log it
        self.get_logger().info(f"Speech Probability: {speech_prob:.4f}")

        # Publish as Int32 (percentage)
        msg_out = Int32()
        msg_out.data = int(speech_prob * 100)  # scale 0–100 %
        self.publisher.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = SoundScore()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
