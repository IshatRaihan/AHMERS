import tkinter
from tkinter import messagebox
import socket
import webbrowser

root = tkinter.Tk()
root.withdraw()

path = 'Log.txt'
file = open(path, 'a')


s = socket.socket()          # Create a socket object
host = socket.gethostname()  # Get local machine name
port = 9000                  # Reserve a port for your service.
s.bind((host, port))         # Bind to the port
print("\nA  H  M  E  R  S\nData Server")
print("\n")
print("Received data from Client 1:\nHeart Beat:      Temperature:")
s.listen(5)
c, addr = s.accept()  # Establish connection with client.


while True:
    data = c.recv(4096)
    res = data.decode("utf8")
    datalist = res.split(';')
    if datalist[0] == '-1':
        Long = datalist[3][0:7]
        messagebox.showwarning("Warning! Client Request Received",
                               "Clinet 1:\nPlease send help!\nAltitude: " + datalist[1] + "m\nLatitude: " + datalist[
                                   2] + "°\nLongitude: " + Long + "°\n")
        url = "https://www.google.com/maps/search/?api=1&query=" + str(datalist[2]) + ", " + str(Long)
        webbrowser.open_new_tab(url)
        print("Client 1:" + "\nPlease send help!")
        print("Altitude: " + datalist[1] + "m")
        print("Latitude: " + datalist[2] + "°")
        print("Longitude: " + Long + "°\n")
    elif datalist[0] == '-2':
        c, addr = s.accept()
    else:
        print(datalist[0] + "bpm" + "            " + datalist[1] + "°C" + "\n")
        file.write(datalist[0] + "bpm" + "   " + datalist[1] + "°C" + "\n")

c.close()
cur.close()
