from google import genai
from google.genai import types


def generate():
    client = genai.Client(
        api_key="AIzaSyDbZKle0m91csH1Gq4h5h2RigQkqsC0lxs"
    )

    model = "gemini-2.5-pro-exp-03-25"
    contents = [
        types.Content(
            role="user",
            parts=[
                types.Part.from_text(text=get_task_breakdown()),
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


def get_task_breakdown():
    task = "Pick up the black bowl from table center and place it on the plate"
    known_skills = ["Pick", "Place", "Open"]
    prompt = f"""
    Given a task, break it down into specific subtasks that can be accomplished using the skills provided below. 
    For each subtask, state the subtask clearly and list the skill needed to achieve the subtask.

    For each subtask, output a concise description of the action and the skill that is required in JSON format.

    Constraints:
    * Each subtask should only use one skill.
    * Subtasks should be ordered by dependency.
    * Only use known skills to perform a subtask.
    * Do not create new skills.
    * If you can not perform a subtask with the skills provided, do not include the subtask.
    * Generate only the subtask and skill. Do not generate any filler text.

    <known_skills>
    {', '.join(known_skills)}
    </known_skills>
    <task>
    {task}
    </task>
    """

    return prompt


if __name__ == "__main__":
    generate()