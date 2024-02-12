import urllib.request, json
while True:
    #get remote json
    with urllib.request.urlopen("http://wpilibpi.local:8080/home/pi/data.json") as url:
        #save it under "data" variable
        data = json.load(url)

    #w signifies write mode, write it to local so it is actully acessable by java
    with open("test.json", "w") as outfile:
        json.dump(data,outfile)
