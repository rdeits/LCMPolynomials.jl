using ZMQ

ctx = Context()
sock = Socket(ctx, REQ)
ZMQ.connect(sock, "tcp://127.0.0.1:6000")

while true
    ZMQ.send(sock, "hello", true)
    ZMQ.send(sock, zeros(UInt8, 5), false)
    ZMQ.recv(sock)
end
