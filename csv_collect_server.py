import socket
import sys
from thread import *
import datetime
 
HOST = ''   # Symbolic name meaning all available interfaces
PORT = 5000 # Arbitrary non-privileged port
 
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print 'Socket created'
 
#Bind socket to local host and port
try:
    s.bind((HOST, PORT))
except socket.error , msg:
    print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    sys.exit()
     
print 'Socket bind complete'
 
#Start listening on socket
s.listen(10) #Can have 10 sockets max
print 'Socket now listening'
 
#Function for handling connections. This will be used to create threads
def clientthread(conn):
    now = datetime.datetime.now()
    filename = now.strftime("%Y-%m-%d_%H-%M") + ".csv"
    print "Saving data to file " + filename
    file = open(filename, "w")
    #header = "Time,Date,Latitude,Longitude,Ground Speed,Heading,Altitude,Roll,Pitch,Yaw,Air Speed"
    #file.write(header + "\n")
    
    #infinite loop so that function do not terminate and thread do not end.
    while True:       
        #Receiving from client
        data = conn.recv(1024)
        if not data: 
            file.close()
            print "Disconnecting from client."
            break
        else:
            file.write(data)
            print data
        
    conn.close()
 
#now keep talking with the client
while 1:
    #wait to accept a connection - blocking call
    conn, addr = s.accept()
    print 'Connected with ' + addr[0] + ':' + str(addr[1])
     
    #start new thread takes 1st argument as a function name to be run, second is the tuple of arguments to the function.
    start_new_thread(clientthread ,(conn,))
 
s.close()