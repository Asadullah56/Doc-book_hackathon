"""
AudioPreprocessor - Audio Processing Module for Robotics Applications

This module provides audio preprocessing capabilities including noise reduction,
voice activity detection, and audio normalization for use with speech recognition
systems in robotics applications.

Author: Humanoid Academy
Date: December 2025
"""

import numpy as np
from scipy import signal
from scipy.signal import butter, filtfilt, welch
import librosa
from typing import Optional, Tuple, Union
import warnings
warnings.filterwarnings('ignore')


class AudioPreprocessor:
    """
    Audio preprocessing class for robotics applications.
    Handles noise reduction, voice activity detection, and audio normalization.
    """

    def __init__(self, sample_rate: int = 16000, chunk_size: int = 1024,
                 noise_reduction_strength: float = 0.8):
        """
        Initialize the audio preprocessor.

        Args:
            sample_rate: Audio sample rate in Hz
            chunk_size: Size of audio chunks to process
            noise_reduction_strength: Strength of noise reduction (0.0 to 1.0)
        """
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.noise_reduction_strength = noise_reduction_strength

        # Initialize noise profile (will be updated during operation)
        self.noise_profile = None
        self.noise_profile_initialized = False

        # Create filter coefficients for audio filtering
        self.low_pass_filter = self._create_low_pass_filter(3000, sample_rate)
        self.high_pass_filter = self._create_high_pass_filter(100, sample_rate)

        # Parameters for voice activity detection
        self.vad_threshold = 0.02  # Energy threshold for VAD
        self.silence_threshold = 0.005  # Lower threshold for silence detection
        self.frame_size = int(0.025 * sample_rate)  # 25ms frames
        self.frame_step = int(0.01 * sample_rate)   # 10ms step

        # Parameters for noise reduction
        self.fft_size = 512
        self.overlap = 0.75
        self.hop_length = int(self.fft_size * (1 - self.overlap))

    def _create_low_pass_filter(self, cutoff_freq: float, sample_rate: int,
                                order: int = 6) -> np.ndarray:
        """
        Create low-pass filter coefficients.

        Args:
            cutoff_freq: Cutoff frequency in Hz
            sample_rate: Sample rate in Hz
            order: Filter order

        Returns:
            Filter coefficients
        """
        nyquist = 0.5 * sample_rate
        normal_cutoff = cutoff_freq / nyquist
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        return b, a

    def _create_high_pass_filter(self, cutoff_freq: float, sample_rate: int,
                                 order: int = 6) -> np.ndarray:
        """
        Create high-pass filter coefficients.

        Args:
            cutoff_freq: Cutoff frequency in Hz
            sample_rate: Sample rate in Hz
            order: Filter order

        Returns:
            Filter coefficients
        """
        nyquist = 0.5 * sample_rate
        normal_cutoff = cutoff_freq / nyquist
        b, a = butter(order, normal_cutoff, btype='high', analog=False)
        return b, a

    def apply_filter(self, audio_data: np.ndarray, filter_coeffs: Tuple) -> np.ndarray:
        """
        Apply a filter to audio data.

        Args:
            audio_data: Input audio data
            filter_coeffs: Filter coefficients (b, a)

        Returns:
            Filtered audio data
        """
        b, a = filter_coeffs
        filtered_data = filtfilt(b, a, audio_data)
        return filtered_data

    def estimate_noise_profile(self, audio_data: np.ndarray,
                              noise_duration: float = 1.0) -> np.ndarray:
        """
        Estimate noise profile from a segment of audio that contains only noise.

        Args:
            audio_data: Audio data containing noise
            noise_duration: Duration of noise segment in seconds

        Returns:
            Estimated noise profile
        """
        # Calculate number of samples for noise duration
        num_samples = int(noise_duration * self.sample_rate)

        # Take the first segment as noise reference
        if len(audio_data) >= num_samples:
            noise_segment = audio_data[:num_samples]
        else:
            noise_segment = audio_data

        # Compute spectral features for noise profile
        noise_spectrum = np.abs(np.fft.rfft(noise_segment, n=self.fft_size))
        self.noise_profile = noise_spectrum
        self.noise_profile_initialized = True

        return self.noise_profile

    def spectral_subtraction(self, audio_data: np.ndarray) -> np.ndarray:
        """
        Apply spectral subtraction noise reduction.

        Args:
            audio_data: Input audio data

        Returns:
            Noise-reduced audio data
        """
        if not self.noise_profile_initialized:
            return audio_data  # Return original if no noise profile available

        # Pad audio to be compatible with FFT size
        remainder = len(audio_data) % self.fft_size
        if remainder != 0:
            padding = self.fft_size - remainder
            audio_data = np.pad(audio_data, (0, padding), mode='constant')

        # Process in overlapping frames
        processed_frames = []
        for i in range(0, len(audio_data) - self.fft_size, self.hop_length):
            frame = audio_data[i:i + self.fft_size]

            # Compute FFT
            frame_fft = np.fft.rfft(frame)
            magnitude = np.abs(frame_fft)
            phase = np.angle(frame_fft)

            # Get corresponding noise profile
            noise_mag = np.interp(
                np.arange(len(magnitude)),
                np.arange(len(self.noise_profile)),
                self.noise_profile
            )

            # Apply spectral subtraction
            enhanced_magnitude = np.maximum(
                magnitude - self.noise_reduction_strength * noise_mag,
                0.1 * magnitude  # Preserve some original signal
            )

            # Reconstruct with original phase
            enhanced_fft = enhanced_magnitude * np.exp(1j * phase)
            enhanced_frame = np.fft.irfft(enhanced_fft)

            processed_frames.append(enhanced_frame)

        # Overlap-add reconstruction
        if processed_frames:
            result = np.zeros_like(audio_data)
            frame_count = 0
            for i in range(0, len(audio_data) - self.fft_size, self.hop_length):
                if frame_count < len(processed_frames):
                    # Apply windowing to reduce artifacts
                    window = np.hanning(self.fft_size)
                    result[i:i + self.fft_size] += (
                        processed_frames[frame_count] * window
                    )
                    frame_count += 1

            # Normalize to prevent clipping
            max_val = np.max(np.abs(result))
            if max_val > 0:
                result = result * 0.9 / max_val

            return result[:len(audio_data)]  # Return to original length

        return audio_data

    def voice_activity_detection(self, audio_data: np.ndarray) -> bool:
        """
        Detect voice activity in audio data using energy-based method.

        Args:
            audio_data: Input audio data

        Returns:
            True if voice activity detected, False otherwise
        """
        # Calculate energy of the signal
        energy = np.mean(audio_data ** 2)

        # Apply threshold-based VAD
        return energy > self.vad_threshold

    def compute_spectral_features(self, audio_data: np.ndarray) -> dict:
        """
        Compute spectral features for audio analysis.

        Args:
            audio_data: Input audio data

        Returns:
            Dictionary of spectral features
        """
        features = {}

        # Compute FFT
        fft = np.fft.rfft(audio_data, n=self.fft_size)
        magnitude = np.abs(fft)

        # Spectral centroid (measure of brightness)
        freqs = np.fft.rfftfreq(self.fft_size, 1.0/self.sample_rate)
        spectral_centroid = np.sum(freqs * magnitude) / (np.sum(magnitude) + 1e-8)
        features['spectral_centroid'] = spectral_centroid

        # Spectral rolloff (frequency below which 85% of energy is contained)
        cumsum = np.cumsum(magnitude)
        rolloff_idx = np.where(cumsum >= 0.85 * cumsum[-1])[0]
        if len(rolloff_idx) > 0:
            spectral_rolloff = freqs[rolloff_idx[0]]
        else:
            spectral_rolloff = 0
        features['spectral_rolloff'] = spectral_rolloff

        # Zero crossing rate
        zcr = np.mean(
            np.abs(np.diff(np.sign(audio_data))) / 2
        )
        features['zero_crossing_rate'] = zcr

        # Fundamental frequency (pitch)
        try:
            f0, _, _ = librosa.pyin(
                audio_data.astype(np.float32),
                fmin=50,
                fmax=400,
                sr=self.sample_rate
            )
            # Take the first non-NaN value
            f0_clean = f0[~np.isnan(f0)]
            if len(f0_clean) > 0:
                features['fundamental_frequency'] = np.mean(f0_clean)
            else:
                features['fundamental_frequency'] = 0
        except:
            features['fundamental_frequency'] = 0

        return features

    def normalize_audio(self, audio_data: np.ndarray,
                      target_level: float = 0.8) -> np.ndarray:
        """
        Normalize audio to target level.

        Args:
            audio_data: Input audio data
            target_level: Target normalization level (0.0 to 1.0)

        Returns:
            Normalized audio data
        """
        # Calculate current RMS
        rms = np.sqrt(np.mean(audio_data ** 2))

        if rms > 0:
            # Calculate normalization factor
            normalization_factor = (target_level * 32767) / np.max(np.abs(audio_data))
            normalized_audio = audio_data * normalization_factor

            # Ensure values are within 16-bit range
            normalized_audio = np.clip(normalized_audio, -32768, 32767)
        else:
            normalized_audio = audio_data

        return normalized_audio

    def apply_compression(self, audio_data: np.ndarray,
                         threshold: float = 0.5, ratio: float = 2.0) -> np.ndarray:
        """
        Apply dynamic range compression to audio.

        Args:
            audio_data: Input audio data
            threshold: Compression threshold (0.0 to 1.0)
            ratio: Compression ratio

        Returns:
            Compressed audio data
        """
        # Normalize audio to 0-1 range for processing
        max_val = np.max(np.abs(audio_data))
        if max_val > 0:
            normalized = audio_data / max_val
        else:
            return audio_data

        # Apply compression
        compressed = np.where(
            np.abs(normalized) <= threshold,
            normalized,  # No compression below threshold
            np.sign(normalized) * (
                threshold + (np.abs(normalized) - threshold) / ratio
            )
        )

        # Restore original amplitude range
        return compressed * max_val

    def process_audio(self, audio_data: np.ndarray) -> Optional[np.ndarray]:
        """
        Complete audio preprocessing pipeline.

        Args:
            audio_data: Input audio data

        Returns:
            Processed audio data or None if no voice activity detected
        """
        if len(audio_data) == 0:
            return None

        # Step 1: Apply high-pass and low-pass filtering to remove DC offset and high frequency noise
        filtered_audio = self.apply_filter(audio_data, self.high_pass_filter)
        filtered_audio = self.apply_filter(filtered_audio, self.low_pass_filter)

        # Step 2: Check for voice activity
        if not self.voice_activity_detection(filtered_audio):
            # No voice activity detected, return None
            return None

        # Step 3: Apply noise reduction (if noise profile is available)
        if self.noise_profile_initialized:
            denoised_audio = self.spectral_subtraction(filtered_audio)
        else:
            denoised_audio = filtered_audio

        # Step 4: Apply dynamic range compression
        compressed_audio = self.apply_compression(denoised_audio)

        # Step 5: Normalize audio
        normalized_audio = self.normalize_audio(compressed_audio)

        # Step 6: Validate output (check for clipping or other artifacts)
        if np.max(np.abs(normalized_audio)) > 0.95 * 32767:
            # Apply soft limiting to prevent clipping
            normalized_audio = np.tanh(normalized_audio / (0.95 * 32767)) * (0.95 * 32767)

        return normalized_audio.astype(np.int16)

    def update_noise_profile(self, audio_data: np.ndarray) -> None:
        """
        Update the noise profile with new audio data (use during known silent periods).

        Args:
            audio_data: Audio data containing noise/silence
        """
        self.estimate_noise_profile(audio_data)