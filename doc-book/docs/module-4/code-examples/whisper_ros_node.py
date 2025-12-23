#!/usr/bin/env python3
"""
WhisperROSNode - ROS 2 Node for OpenAI Whisper Integration

This node provides real-time speech-to-text capabilities using OpenAI's Whisper API,
integrated with ROS 2 for robotics applications. It handles audio preprocessing,
real-time streaming, and voice command interpretation for humanoid robots.

Author: Humanoid Academy
Date: December 2025
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
import openai
import numpy as np
import pyaudio
import wave
import threading
import queue
import time
from typing import Optional, Dict, Any
import os
from .audio_preprocessor import AudioPreprocessor


class WhisperROSNode(Node):
    """
    ROS 2 Node that integrates OpenAI Whisper API for real-time speech recognition.
    Processes audio input and publishes transcribed text to ROS topics.
    """

    def __init__(self):
        super().__init__('whisper_ros_node')

        # Initialize parameters
        self.declare_parameter('whisper_model', 'whisper-1')
        self.declare_parameter('audio_sample_rate', 16000)
        self.declare_parameter('audio_chunk_size', 1024)
        self.declare_parameter('vad_threshold', 0.3)  # Voice Activity Detection threshold
        self.declare_parameter('silence_timeout', 2.0)  # Timeout for silence detection
        self.declare_parameter('api_key', os.getenv('OPENAI_API_KEY', ''))

        # Get parameters
        self.whisper_model = self.get_parameter('whisper_model').value
        self.sample_rate = self.get_parameter('audio_sample_rate').value
        self.chunk_size = self.get_parameter('audio_chunk_size').value
        self.vad_threshold = self.get_parameter('vad_threshold').value
        self.silence_timeout = self.get_parameter('silence_timeout').value
        self.api_key = self.get_parameter('api_key').value

        # Validate API key
        if not self.api_key:
            self.get_logger().error("OpenAI API key not found. Please set OPENAI_API_KEY environment variable.")
            raise ValueError("OpenAI API key is required")

        # Set OpenAI API key
        openai.api_key = self.api_key

        # Initialize audio processing components
        self.audio_preprocessor = AudioPreprocessor(
            sample_rate=self.sample_rate,
            chunk_size=self.chunk_size
        )

        # Audio buffer for streaming
        self.audio_buffer = queue.Queue()
        self.is_listening = False
        self.listening_thread = None

        # ROS 2 Publishers
        self.transcript_pub = self.create_publisher(
            String,
            'voice_transcript',
            QoSProfile(depth=10)
        )

        self.interpretation_pub = self.create_publisher(
            String,
            'voice_interpretation',
            QoSProfile(depth=10)
        )

        # ROS 2 Subscribers
        self.audio_sub = self.create_subscription(
            AudioData,
            'audio_input',
            self.audio_callback,
            QoSProfile(depth=10)
        )

        # Service clients for coordination with other modules
        self.get_logger().info(f"WhisperROSNode initialized with model: {self.whisper_model}")

        # Statistics
        self.stats = {
            'total_requests': 0,
            'total_errors': 0,
            'avg_response_time': 0.0,
            'last_request_time': time.time()
        }

        # Create timer for periodic statistics reporting
        self.stats_timer = self.create_timer(30.0, self.report_statistics)

    def audio_callback(self, msg: AudioData):
        """
        Callback function for audio input messages.
        Adds audio data to the processing queue.
        """
        try:
            # Convert audio data to numpy array
            audio_array = np.frombuffer(msg.data, dtype=np.int16)

            # Preprocess audio (noise reduction, normalization, VAD)
            processed_audio = self.audio_preprocessor.process_audio(audio_array)

            # Add processed audio to buffer for Whisper processing
            if processed_audio is not None:
                self.audio_buffer.put(processed_audio)

                # Start processing if not already running
                if not self.is_listening:
                    self.start_listening()

        except Exception as e:
            self.get_logger().error(f"Error in audio callback: {str(e)}")
            self.stats['total_errors'] += 1

    def start_listening(self):
        """Start the listening thread for continuous audio processing."""
        if not self.is_listening:
            self.is_listening = True
            self.listening_thread = threading.Thread(target=self.process_audio_stream)
            self.listening_thread.daemon = True
            self.listening_thread.start()
            self.get_logger().info("Started audio processing thread")

    def process_audio_stream(self):
        """Process audio stream and send to Whisper API."""
        accumulated_audio = np.array([], dtype=np.int16)
        silence_start_time = None
        last_activity_time = time.time()

        while self.is_listening:
            try:
                # Check for accumulated audio
                if not self.audio_buffer.empty():
                    audio_chunk = self.audio_buffer.get_nowait()
                    accumulated_audio = np.concatenate([accumulated_audio, audio_chunk])
                    last_activity_time = time.time()
                    silence_start_time = None
                else:
                    # Check for silence timeout
                    current_time = time.time()
                    if len(accumulated_audio) > 0:
                        if current_time - last_activity_time > self.silence_timeout:
                            if silence_start_time is None:
                                silence_start_time = current_time
                            elif current_time - silence_start_time > 0.5:  # Additional short timeout
                                # Send accumulated audio for transcription
                                self.transcribe_audio(accumulated_audio)
                                accumulated_audio = np.array([], dtype=np.int16)
                                silence_start_time = None
                                last_activity_time = current_time
                    else:
                        time.sleep(0.01)  # Small delay to prevent busy waiting
                        continue

                # Process accumulated audio if it reaches a reasonable size
                if len(accumulated_audio) >= self.sample_rate * 2:  # 2 seconds of audio
                    self.transcribe_audio(accumulated_audio)
                    accumulated_audio = np.array([], dtype=np.int16)
                    last_activity_time = time.time()

            except queue.Empty:
                time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f"Error in audio stream processing: {str(e)}")
                self.stats['total_errors'] += 1
                time.sleep(0.01)

    def transcribe_audio(self, audio_data: np.ndarray):
        """
        Send audio data to Whisper API for transcription.

        Args:
            audio_data: Numpy array containing audio samples
        """
        try:
            start_time = time.time()

            # Convert numpy array to WAV format for Whisper API
            wav_buffer = self.audio_array_to_wav(audio_data)

            # Send to Whisper API
            response = openai.Audio.transcribe(
                model=self.whisper_model,
                file=wav_buffer,
                response_format="text"
            )

            transcript = response.strip()

            if transcript:  # Only publish if there's actual content
                # Create and publish transcript message
                transcript_msg = String()
                transcript_msg.data = transcript
                self.transcript_pub.publish(transcript_msg)

                # Log the transcription
                self.get_logger().info(f"Transcribed: {transcript}")

                # Process interpretation (send to cognitive planning module)
                self.process_interpretation(transcript)

                # Update statistics
                response_time = time.time() - start_time
                self.stats['total_requests'] += 1
                self.stats['avg_response_time'] = (
                    (self.stats['avg_response_time'] * (self.stats['total_requests'] - 1) + response_time)
                    / self.stats['total_requests']
                )
                self.stats['last_request_time'] = time.time()

        except Exception as e:
            self.get_logger().error(f"Error in transcription: {str(e)}")
            self.stats['total_errors'] += 1

    def audio_array_to_wav(self, audio_array: np.ndarray) -> bytes:
        """
        Convert numpy audio array to WAV format bytes for Whisper API.

        Args:
            audio_array: Numpy array of audio samples

        Returns:
            Bytes object containing WAV data
        """
        import io
        import wave

        # Normalize audio to prevent clipping
        audio_normalized = np.clip(audio_array, -32768, 32767)

        # Create WAV file in memory
        wav_buffer = io.BytesIO()
        with wave.open(wav_buffer, 'wb') as wav_file:
            wav_file.setnchannels(1)  # Mono
            wav_file.setsampwidth(2)  # 16-bit
            wav_file.setframerate(self.sample_rate)
            wav_file.writeframes(audio_normalized.astype(np.int16).tobytes())

        # Reset buffer position
        wav_buffer.seek(0)
        return wav_buffer

    def process_interpretation(self, transcript: str):
        """
        Process the transcript for cognitive interpretation.
        This could involve sending to an LLM for task decomposition.

        Args:
            transcript: The transcribed voice command
        """
        try:
            # In a real implementation, this would send to the cognitive planning module
            # For now, we'll just publish a simple interpretation
            interpretation_msg = String()
            interpretation_msg.data = f"Interpreted command: {transcript}"
            self.interpretation_pub.publish(interpretation_msg)

            self.get_logger().info(f"Published interpretation: {transcript}")

        except Exception as e:
            self.get_logger().error(f"Error in interpretation processing: {str(e)}")

    def report_statistics(self):
        """Report periodic statistics about Whisper API usage."""
        self.get_logger().info(
            f"Whisper API Statistics - "
            f"Requests: {self.stats['total_requests']}, "
            f"Errors: {self.stats['total_errors']}, "
            f"Avg Response Time: {self.stats['avg_response_time']:.2f}s"
        )

    def destroy_node(self):
        """Clean up resources when node is destroyed."""
        self.is_listening = False
        if self.listening_thread and self.listening_thread.is_alive():
            self.listening_thread.join(timeout=2.0)

        super().destroy_node()


def main(args=None):
    """Main function to run the WhisperROSNode."""
    rclpy.init(args=args)

    try:
        node = WhisperROSNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error running WhisperROSNode: {str(e)}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()