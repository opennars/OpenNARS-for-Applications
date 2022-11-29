import openai

openai.api_key = "PUT_YOUR_API_KEY"
gpt_prompt = """
Q: Jack is a cat
A: < { Jack } --> cat > .

Q: cats are animals
A: < cat --> animal > .

Q: dogs are barking
A: < dog --> [ bark ] > .

Q: Garfield is a yellow cat
A: < { Garfield } --> ( [ yellow ] & cat ) > .

Q: slow cars are bad
A: < ( [ slow ] & car ) --> [ bad ] > .

Q: Tim is in the garden
A:  < ( { Tim } * garden ) --> in > .

Q: cats eats mice
A: < ( cat * mouse ) --> eat > .

Q: the human drinks white liquid
A: < ( human * ( [ white ] & liquid ) ) --> drink > .

Q: Cats are furry?
A: < cat --> [ furry ] > ?

Q: The cat is in the garden
A: < ( cat * garden ) --> in > .

Q: cats are furry animals
A: < cat --> ( [ furry ] & animal ) > .

Q: what is a cat?
A: < ?1 --> cat > ?

Q: who likes animals?
A: < ( ?1 * animal ) --> like > ?

"""

while True:
    inp = input().rstrip("\n")
    if len(inp) == 0:
        print("\n")
    elif inp.isdigit() or inp.startswith("*") or inp.startswith("(") or inp.startswith("<"):
        print(inp)
    else:
        gpt_prompt_with_input = gpt_prompt + "Q: " + inp
        response = openai.Completion.create(
          engine="text-davinci-002",
          prompt=gpt_prompt_with_input,
          temperature=0.5,
          max_tokens=100, #256
          top_p=1.0,
          frequency_penalty=0.0,
          presence_penalty=0.0
        )
        print(response['choices'][0]['text'].split("A: ")[1])
