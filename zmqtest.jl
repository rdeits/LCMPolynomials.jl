using ZMQ

ctx = Context()
sock = Socket(ctx, REQ)
ZMQ.connect(sock, "tcp://127.0.0.1:5555")

# const m1 = "hello"
# const m2 = zeros(UInt8, 5)
const box = Ref{Tuple{Any, Any}}()

while true
    m1 = "hello"
    m2 = zeros(UInt8, 5)
    # ZMQ.send(sock, m1, true)
    ZMQ.send(sock, m2, false)
    box[] = (m1, m2)
    ZMQ.recv(sock)
end
