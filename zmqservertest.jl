using ZMQ

ctx = Context()
sock = Socket(ctx, REP)
ZMQ.bind(sock, "tcp://127.0.0.1:6000")

function recv_multipart(sock::ZMQ.Socket)
    frames = [ZMQ.recv(sock)]
    while ZMQ.ismore(sock)
        push!(frames, ZMQ.recv(sock))
    end
    frames
end

while true
    frames = recv_multipart(sock)
    cmd = unsafe_string(frames[1])
    ZMQ.send(sock, "error: unrecognized command")
end
