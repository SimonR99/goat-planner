import queue

import librosa
import noisereduce as nr
import numpy as np
import sounddevice as sd
from faster_whisper import WhisperModel
from transformers import pipeline
from transformers.pipelines.audio_utils import ffmpeg_microphone_live


class VoiceAssistant:
    def __init__(self, whisper_model="base", language="en", device="cuda"):
        """
        Initialize the voice assistant with Faster Whisper and audio classification for wake word detection.

        Parameters:
        - whisper_model: Whisper model size ('tiny', 'base', 'small', 'medium', 'large').
        - language: Language code for transcription (e.g., 'en' for English).
        - device: Device to run the pipeline ('cuda' for GPU, 'cpu' for CPU).
        """
        self.device = device
        self.language = language
        print(f"Loading Faster Whisper model ({whisper_model})...")
        self.model = WhisperModel(whisper_model, device=device)
        print("Loading wake word detection model...")
        self.wake_word_classifier = pipeline(
            "audio-classification",
            model="MIT/ast-finetuned-speech-commands-v2",
            device=device,
        )
        self.audio_queue = queue.Queue()
        self.is_listening = False
        print("Initialization complete.")

    def preprocess_audio(self, audio, sr):
        """
        Preprocess the audio: resampling and noise reduction.

        Parameters:
        - audio: Input audio signal.
        - sr: Sampling rate.

        Returns:
        - Preprocessed audio signal.
        """
        audio = librosa.resample(audio, orig_sr=sr, target_sr=16000)
        audio = nr.reduce_noise(y=audio, sr=16000)
        return audio

    def stt(self, audio):
        """
        Perform speech-to-text using Faster Whisper.

        Parameters:
        - audio: Input audio signal (1D numpy array).

        Returns:
        - Transcribed text.
        """
        audio = audio.astype(np.float32)  # Ensure correct dtype
        segments, _ = self.model.transcribe(audio, beam_size=5, language=self.language)
        transcription = " ".join([segment.text for segment in segments])
        return transcription

    def detect_wake_word(self, audio_chunk, wake_word="marvin", prob_threshold=0.5):
        """
        Detect if the wake word is present in the audio chunk.

        Parameters:
        - audio_chunk: Input audio signal (1D numpy array).
        - wake_word: The wake word to detect.
        - prob_threshold: Probability threshold for detection.

        Returns:
        - True if the wake word is detected, False otherwise.
        """
        predictions = self.wake_word_classifier(audio_chunk)
        for prediction in predictions:
            if (
                prediction["label"] == wake_word
                and prediction["score"] > prob_threshold
            ):
                return True
        return False

    def detect_silence(self, audio, threshold=0.01, min_silence_len=0.5, sr=16000):
        """
        Detect if the audio segment is silent.

        Parameters:
        - audio: Input audio signal (1D numpy array).
        - threshold: Silence threshold.
        - min_silence_len: Minimum silence duration in seconds.
        - sr: Sampling rate.

        Returns:
        - True if the segment is silent, False otherwise.
        """
        frame_size = int(min_silence_len * sr)
        rms = np.sqrt(np.mean(audio**2))
        return rms < threshold

    def audio_callback(self, indata, frames, time, status):
        """
        Callback function for audio stream.
        """
        if status:
            print(f"Status: {status}")
        self.audio_queue.put(indata.copy())

    def process_audio_stream(self, wake_word="marvin", trigger_words=None):
        """
        Process the continuous audio stream.
        """
        try:
            with sd.InputStream(
                callback=self.audio_callback,
                channels=1,
                samplerate=16000,
                blocksize=32000,  # 2-second blocks
            ):

                print("Listening for wake word... Press Ctrl+C to stop.")
                triggered_mode = False

                while self.is_listening:
                    audio_block = self.audio_queue.get().flatten()
                    audio_block = self.preprocess_audio(audio_block, sr=16000)

                    if not triggered_mode:
                        # Listen for wake word
                        if self.detect_wake_word(audio_block, wake_word=wake_word):
                            print(f"Wake word '{wake_word}' detected!")
                            triggered_mode = True
                    else:
                        # Process speech until silence
                        if self.detect_silence(audio_block):
                            print("Silence detected, returning to wake word detection.")
                            triggered_mode = False
                        else:
                            transcription = self.stt(audio_block)
                            print(f"Processing speech: {transcription}")
                            # Here you can add text-to-speech processing

        except KeyboardInterrupt:
            print("\nStopping the voice assistant...")
            self.is_listening = False
        except Exception as e:
            print(f"Error in audio processing: {e}")
            self.is_listening = False

    def start(self, wake_word="marvin", trigger_words=None):
        """
        Start the voice assistant in continuous listening mode.
        """
        self.is_listening = True
        self.process_audio_stream(wake_word=wake_word, trigger_words=trigger_words)

    def stop(self):
        """
        Stop the voice assistant.
        """
        self.is_listening = False


if __name__ == "__main__":
    # Example usage
    assistant = VoiceAssistant(whisper_model="base", language="en", device="cuda")
    wake_word = "marvin"
    try:
        assistant.start(wake_word=wake_word)
    except KeyboardInterrupt:
        assistant.stop()
