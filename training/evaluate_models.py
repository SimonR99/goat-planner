from transformers import AutoModelForCausalLM, AutoTokenizer, BitsAndBytesConfig
import torch
import json
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Dict, Any
import wandb
from pathlib import Path

class ModelSize(Enum):
    LLAMA_1B = "meta-llama/Llama-3.2-1B-Instruct"
    LLAMA_3B = "meta-llama/Llama-3.2-3B-Instruct"

class TrainingMethod(Enum):
    LORA = "lora"
    IA3 = "ia3"

class OutputFormat(Enum):
    XML = "xml"
    JSON = "json"

@dataclass
class ModelConfig:
    size: ModelSize
    method: TrainingMethod
    format: OutputFormat
    checkpoint_path: str

def load_model_and_tokenizer(config: ModelConfig):
    """Load a trained model and its tokenizer"""
    torch_dtype = torch.float16
    attn_implementation = "eager"

    bnb_config = BitsAndBytesConfig(
        load_in_4bit=True,
        bnb_4bit_quant_type="nf4",
        bnb_4bit_compute_dtype=torch_dtype,
        bnb_4bit_use_double_quant=True,
    )

    model = AutoModelForCausalLM.from_pretrained(
        config.size.value,
        quantization_config=bnb_config,
        device_map="auto",
        attn_implementation=attn_implementation
    )

    # Load the trained weights
    model.load_state_dict(torch.load(config.checkpoint_path))
    
    tokenizer = AutoTokenizer.from_pretrained(config.size.value, trust_remote_code=True)
    tokenizer.add_special_tokens({'pad_token': '[PAD]'})

    return model, tokenizer

def load_templates():
    """Load all necessary templates and data"""
    with open('data/templates.json', 'r') as f:
        templates = json.load(f)
    
    with open('data/actions.json', 'r') as f:
        actions = json.load(f)
    
    with open('data/objects.json', 'r') as f:
        objects = json.load(f)

    return templates, actions['action_list'], objects['object_list']

def evaluate_all_models(models_dir: str = "checkpoints", results_dir: str = "results"):
    """Evaluate all trained models and save results"""
    # Initialize wandb for logging
    wandb.init(project="bt-evaluation")

    # Create results directory
    Path(results_dir).mkdir(exist_ok=True)

    # Load templates and data
    templates, action_list, object_list = load_templates()

    # Define model configurations to evaluate
    configs = [
        ModelConfig(
            size=ModelSize.LLAMA_1B,
            method=TrainingMethod.LORA,
            format=OutputFormat.JSON,
            checkpoint_path=f"{models_dir}/llama-1b-bt-json-lora/pytorch_model.bin"
        ),
        ModelConfig(
            size=ModelSize.LLAMA_1B,
            method=TrainingMethod.LORA,
            format=OutputFormat.XML,
            checkpoint_path=f"{models_dir}/llama-1b-bt-xml-lora/pytorch_model.bin"
        ),
        ModelConfig(
            size=ModelSize.LLAMA_1B,
            method=TrainingMethod.IA3,
            format=OutputFormat.JSON,
            checkpoint_path=f"{models_dir}/llama-1b-bt-json-ia3/pytorch_model.bin"
        ),
        ModelConfig(
            size=ModelSize.LLAMA_1B,
            method=TrainingMethod.IA3,
            format=OutputFormat.XML,
            checkpoint_path=f"{models_dir}/llama-1b-bt-xml-ia3/pytorch_model.bin"
        ),
        ModelConfig(
            size=ModelSize.LLAMA_3B,
            method=TrainingMethod.LORA,
            format=OutputFormat.JSON,
            checkpoint_path=f"{models_dir}/llama-3b-bt-json-lora/pytorch_model.bin"
        ),
        ModelConfig(
            size=ModelSize.LLAMA_3B,
            method=TrainingMethod.LORA,
            format=OutputFormat.XML,
            checkpoint_path=f"{models_dir}/llama-3b-bt-xml-lora/pytorch_model.bin"
        ),
        ModelConfig(
            size=ModelSize.LLAMA_3B,
            method=TrainingMethod.IA3,
            format=OutputFormat.JSON,
            checkpoint_path=f"{models_dir}/llama-3b-bt-json-ia3/pytorch_model.bin"
        ),
        ModelConfig(
            size=ModelSize.LLAMA_3B,
            method=TrainingMethod.IA3,
            format=OutputFormat.XML,
            checkpoint_path=f"{models_dir}/llama-3b-bt-xml-ia3/pytorch_model.bin"
        ),
    ]

    results = {}

    for config in configs:
        print(f"\nEvaluating {config.size.value} with {config.method.value} for {config.format.value} format")
        
        try:
            # Load model and tokenizer
            model, tokenizer = load_model_and_tokenizer(config)

            # Prepare system prompt based on format
            if config.format == OutputFormat.JSON:
                system_prompt = templates['template'].format(
                    format_type="JSON",
                    example=templates['question_example'] + "\n" + templates['answer_example'] + "\n" + templates['json_example'],
                    available_actions=action_list,
                    object_list=object_list,
                )
            else:
                system_prompt = templates['template'].format(
                    format_type="XML",
                    example=templates['question_example'] + "\n" + templates['answer_example'] + "\n" + templates['xml_example'],
                    available_actions=action_list,
                    object_list=object_list,
                )

            # Evaluate model
            from evaluation import evaluate_model
            result = evaluate_model(
                model=model,
                tokenizer=tokenizer,
                formatting_prompt=templates['training_template'],
                validation_type=config.format.value,
                query_file="./query_dataset.json",
                instruction=system_prompt,
                action_list=action_list
            )

            # Save results
            model_name = f"llama-{config.size.value.split('-')[-2]}-{config.method.value}-{config.format.value}"
            results[model_name] = result

            # Log to wandb
            wandb.log({
                "model": model_name,
                "result": result
            })

            # Save to file
            result_path = Path(results_dir) / f"{model_name}_results.json"
            with open(result_path, 'w') as f:
                json.dump(result, f, indent=2)

        except Exception as e:
            print(f"Error evaluating {config.size.value} with {config.method.value}: {str(e)}")
            continue

        finally:
            # Clear CUDA cache
            torch.cuda.empty_cache()

    return results

if __name__ == "__main__":
    results = evaluate_all_models()
    print("\nEvaluation Results:")
    for model_name, result in results.items():
        print(f"\n{model_name}:")
        print(json.dumps(result, indent=2)) 