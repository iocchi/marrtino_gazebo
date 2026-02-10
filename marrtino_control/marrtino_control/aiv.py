import os
import ai



class AI(ai.AI):

    def __init__(self):
        ai.AI.__init__(self)

    def __del__(self):
        ai.AI.__del__(self)

    def vision(self, image_b64, prompt):
        # img must be a base64 encoding of the image !!!
        try:           
            response = self.client.responses.create(
                model=ai.MODEL,
                input=[
                    {
                    "role": "user",
                    "content": [
                        {
                          "type": "input_text",
                          "text":f"{prompt}"
                        },
                        {
                          "type": "input_image",
                          "image_url": f"data:image/jpeg;base64,{image_b64}"
                        },
                    ],
                    }
                ],
                # temperature=0.1,
                )

            self.gpt_total_tokens += response.usage.total_tokens
            self.gpt_input_tokens += response.usage.input_tokens
            self.gpt_output_tokens += response.usage.output_tokens

            self.logf.write("vision\n")
            self.logf.write(f"{prompt}\n{response.output_text}\n{response.usage.input_tokens};{response.usage.output_tokens}\n----\n")
            self.logf.flush()

            return response.output_text
        except Exception as e:
            print(f"AI vision:: Getting response - {e}")
            return None


    def vision_file(self, img_filepath, prompt):

        image_b64 = None

        # Read current image with the view of the robot
        try:
            with open(img_filepath, "rb") as img_file:
                encoded_image = base64.b64encode(img_file.read()).decode("utf-8")
        except Exception as e:
            print(f"AI vision:: Reading image {img_filepath} - {e}")
            return None

        if image_b64 is None:
            return None
        
        return self.vision(image_b64, prompt)
        
    def query(self, description, query):
        query_system = { 
            'role': 'system',
            'content': "Answer the user request about this description: " + description
        }
        user_message = {
            'role': 'user',
            'content': query,
        }
        self.logf.write("query\n")
        self.logf.write(f"{description}\n")        
        response = self.send_llm_messages([query_system, user_message])
        return response




