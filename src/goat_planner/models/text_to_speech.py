import numpy as np
import sounddevice as sd
import torch
from datasets import load_dataset
from transformers import pipeline


class TextToSpeech:
    def __init__(
        self,
        model_name="microsoft/speecht5_tts",
        dataset_name="Matthijs/cmu-arctic-xvectors",
        speaker_index=7306,
        device=0,
    ):
        """
        Initialize the text-to-speech object.

        Parameters:
        - model_name: Name of the pre-trained TTS model.
        - dataset_name: Name of the dataset with speaker embeddings.
        - device: Device to run the pipeline (0 for GPU, -1 for CPU).
        """
        print("Loading embeddings dataset...")
        self.embeddings_dataset = load_dataset(dataset_name, split="validation")
        print("Loading text-to-speech pipeline...")
        self.pipe = pipeline("text-to-speech", model=model_name, device=device)
        print("Initialization complete.")
        self.speaker_embedding = torch.tensor(
            self.embeddings_dataset[speaker_index]["xvector"]
        ).unsqueeze(0)

    def speak(self, text):
        """
        Generate and play speech for the given text using different speaker embeddings.

        Parameters:
        - text: Text to convert to speech.
        """

        # Generate the speech
        speech = self.pipe(
            text, forward_params={"speaker_embeddings": self.speaker_embedding}
        )

        # Play the audio
        audio = np.array(speech["audio"], dtype=np.float32)
        sd.play(audio, samplerate=speech["sampling_rate"])
        sd.wait()


if __name__ == "__main__":
    # Example usage
    tts = TextToSpeech()
    text = "Hey ! How are you doing ?"
    tts.speak(text)
