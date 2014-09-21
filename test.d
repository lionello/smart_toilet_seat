import std.stdio;
import std.socket;

void main()
{
    auto destIP = new InternetAddress("wc.noun.ly", 5417);

    auto sock = new UdpSocket(AddressFamily.INET);
    sock.sendTo(cast(void[])`{"wc":"udp","ir":11,"gas":123}`, destIP);

    auto sock2 = new TcpSocket(AddressFamily.INET);
    sock2.connect(destIP);
    sock2.send(cast(void[])`{"wc":"tcp","ir":11,"gas":123}`);
}

