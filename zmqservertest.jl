using ZMQ

ctx = Context()
sock = Socket(ctx, REP)
ZMQ.bind(sock, "tcp://127.0.0.1:5555")

while true
    ZMQ.recv(sock)
    while ZMQ.ismore(sock)
        ZMQ.recv(sock)
    end
    ZMQ.send(sock, "ok")
end
