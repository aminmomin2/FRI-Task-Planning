from google import genai
from google.genai import types
from config import Config


def generate(task_prompt):
    """
    Generate a task breakdown using the Gemini model.
    
    Args:
        task_prompt (str): The task description from the user
    """
    client = genai.Client(
        api_key=Config.get_api_key()
    )

    model = "gemini-2.0-flash-thinking-exp"
    contents = [
        types.Content(
            role="user",
            parts=[
                types.Part.from_text(text=get_task_breakdown(task_prompt)),
            ],
        ),
    ]

    generate_content_config = types.GenerateContentConfig(
        response_mime_type="text/plain",
    )

    for chunk in client.models.generate_content_stream(
        model=model,
        contents=contents,
        config=generate_content_config,
    ):
        print(chunk.text, end="")


def get_task_breakdown(task):
    """
    Generate a prompt for task breakdown based on the given task.
    
    Args:
        task (str): The task description from the user
    
    Returns:
        str: Formatted prompt for the model
    """
    known_skills = ["Pick", "Place"]

    prompt = f"""
    You are a task planner for a robot that can only perform the following skills: {', '.join(known_skills)}.

    Given a high-level task, decompose it into a sequence of subtasks. Each subtask must:
    - Use **only one** of the known skills
    - Be ordered logically based on dependencies
    - Be represented as a JSON array of objects
    - Each object should include:
        - "subtask": a concise, clear description of the specific action
        - "skill": one of the known skills (case-sensitive)

    Do **not** add any extra text or explanation.

    <known_skills>
    {', '.join(known_skills)}
    </known_skills>
    <task>
    {task}
    </task>
    """

    return prompt


if __name__ == "__main__":
    # Get task from user input
    task_prompt = input("Enter your task (e.g., 'Stack the red block on top of the blue block'): ")
    generate(task_prompt)