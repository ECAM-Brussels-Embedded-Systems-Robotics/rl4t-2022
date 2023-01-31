import socket
import time

import speech_recognition as sr

h_name = socket.gethostname()           #get the name of the host to obtain his IP adress
ADRESSE = socket.gethostbyname(h_name)
PORT = 8888                              #Definition of the IP adress and the port 
memory = 0

serveur = socket.socket(socket.AF_INET, socket.SOCK_STREAM)     #Création du socket du serveur TCP
serveur.bind((ADRESSE, PORT))                                   #liaison du socket à l'adresse de la machine
serveur.listen(1)                                               #mise sour écoute du socket crée sur l'adresse sur laquelle il a été lié


print(ADRESSE)  #
def recognize_speech_from_mic(recognizer, microphone):
    """Transcribe speech from recorded from `microphone`.

    Returns a dictionary with three keys:
    "success": a boolean indicating whether or not the API request was
               successful
    "error":   `None` if no error occured, otherwise a string containing
               an error message if the API could not be reached or
               speech was unrecognizable
    "transcription": `None` if speech could not be transcribed,
               otherwise a string containing the transcribed text
    """
    # check that recognizer and microphone arguments are appropriate type
    if not isinstance(recognizer, sr.Recognizer):
        raise TypeError("`recognizer` must be `Recognizer` instance")

    if not isinstance(microphone, sr.Microphone):
        raise TypeError("`microphone` must be `Microphone` instance")

    # adjust the recognizer sensitivity to ambient noise and record audio
    # from the microphone
    with microphone as source:
        recognizer.adjust_for_ambient_noise(source)
        recognizer.pause_threshold = 1
        recognizer.energy_threshold = 700
        audio = recognizer.listen(source,phrase_time_limit=2)

    # set up the response object
    response = {
        "success": True,
        "error": None,
        "transcription": None
    }

    # try recognizing the speech in the recording
    # if a RequestError or UnknownValueError exception is caught,
    #     update the response object accordingly
    try:
        response["transcription"] = recognizer.recognize_google(audio)
    except sr.RequestError:
        # API was unreachable or unresponsive
        response["success"] = False
        response["error"] = "API unavailable"
    except sr.UnknownValueError:
        # speech was unintelligible
        response["error"] = "Unable to recognize speech"

    return response



if __name__ == "__main__":
    WORDS = ["move", "back", "left", "right", "stop", "shut down"]
    LOOP = 1
    PROMPT_LIMIT = 5
    
    # create recognizer and mic instances
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()

    # format the instructions string
    instructions = (
        "Try one of these commands to lead the robot:\n"
        "{words}\n"
    ).format(words=', '.join(WORDS))

    print(instructions)
    time.sleep(3)
    while LOOP:
        for j in range(PROMPT_LIMIT):
            print('Speak!')
            command = recognize_speech_from_mic(recognizer, microphone)
            if command["transcription"]:
                break
            if not command["success"]:
                break
            print("I didn't catch that. What did you say?\n")

        # if there was an error, stop
        if command["error"]:
            print("ERROR: {}".format(command["error"]))
        

        # show the user the transcription
        print("You said: {}".format(command["transcription"]))
        
        if format(command["transcription"]) == "shut down":
            LOOP = 0
            client.close()
            serveur.close()
            
        elif format(command["transcription"]) == "move":
            client, adresseClient = serveur.accept()
            command = "1"
            memory = 1
            command = bytes(command,'ASCII')
            n = client.send(command)
            
        elif format(command["transcription"]) == "back":
            client, adresseClient = serveur.accept()
            command = "2"
            memory = 2
            command = bytes(command,'ASCII')
            n = client.send(command)

        elif format(command["transcription"]) == "left":
            client, adresseClient = serveur.accept()
            command = "3"
            command = bytes(command,'ASCII')
            n = client.send(command)
            client, adresseClient = serveur.accept()
            command = "4"
            command = bytes(command,'ASCII')
            n = client.send(command)
            time.sleep(1);
            client, adresseClient = serveur.accept()
            command = "6"
            command = bytes(command,'ASCII')
            n = client.send(command)
            if memory == 1:
                client, adresseClient = serveur.accept()
                command = "1"
                command = bytes(command,'ASCII')
                n = client.send(command)
            elif memory == 2:
                client, adresseClient = serveur.accept()
                command = "2"
                command = bytes(command,'ASCII')
                n = client.send(command)
                
        elif format(command["transcription"]) == "right":
            client, adresseClient = serveur.accept()
            command = "3"
            command = bytes(command,'ASCII')
            n = client.send(command)
            client, adresseClient = serveur.accept()
            command = "5"
            command = bytes(command,'ASCII')
            n = client.send(command)
            time.sleep(1);
            client, adresseClient = serveur.accept()
            command = "6"
            command = bytes(command,'ASCII')
            n = client.send(command)
            if memory == 1:
                client, adresseClient = serveur.accept()
                command = "1"
                command = bytes(command,'ASCII')
                n = client.send(command)
            elif memory == 2:
                client, adresseClient = serveur.accept()
                command = "2"
                command = bytes(command,'ASCII')
                n = client.send(command)    
        else:
            client, adresseClient = serveur.accept()
            command = "0"
            memory = 0
            command = bytes(command,'ASCII')
            n = client.send(command)
            if (n != len(command)):
                print ('Erreur envoi.')
            else:
                print ('Envoi ok.')
