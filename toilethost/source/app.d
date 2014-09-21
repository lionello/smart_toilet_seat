import std.stdio;
import std.socket;
import std.stdio;
import std.json;
import std.concurrency;

enum MAX_PACKET_SIZE = 1400; // <MRU keeps it fast
enum HTTP_PORT = 8888;
enum TOILET_PORT = 5417;

alias JSONObject = JSONValue[string];
shared string log = null;

void append_to_log(in char[] payload)
{
	if (log is null || log.length > 16384)
		log = payload.idup;//TODO: truncate
	else
		log = cast(string)(log ~ (',' ~ payload));//assumeUnique
}

shared Address lastSender;

void payloadhandler_udp()
{
    auto sock = new UdpSocket(AddressFamily.INET);
    sock.bind(new InternetAddress(TOILET_PORT));
    while (1)
    {
		char[MAX_PACKET_SIZE] packet = void;
		Address sender;
    	try
    	{
			const len = sock.receiveFrom(packet, sender);
			if (len > 0)
			{
	    		receive_payload(packet[0..len], sender);
	    	}
    	}
    	catch
    	{
    		//nop
    	}
    }
}

float raw2temp(long raw)
{
	return raw / 7.8f - 4;
}

void receive_payload(in char[] packet, Address sender)
{
	//debug writeln(packet);
	lastSender = cast(shared)sender;
	auto json = parseJSON(packet);
	json.object["C"] = raw2temp(json["heat0"].integer);
	json.object["from"] = sender.toString();
	import std.datetime;
	json.object["at"] = Clock.currTime.toISOExtString();
	append_to_log(json.toString());
//	debug writeln(json);
}


void handle_payload_client(shared Socket c)
{
	auto client = cast(Socket)c;
	scope(exit) client.close();

	char[MAX_PACKET_SIZE] packet = void;
    auto len = client.receive(packet);
    if (len > 0)
    	receive_payload(packet[0..len], client.remoteAddress);
}


void payloadhandler_tcp()
{
    auto sock = new TcpSocket(AddressFamily.INET);
    sock.bind(new InternetAddress(TOILET_PORT));
    sock.listen(5);
    while (1)
    {
    	auto client = sock.accept();
    	spawn(&handle_payload_client, cast(shared)client);
    }
}

void handle_http_client(shared Socket c)
{
	auto client = cast(Socket)c;
	scope(exit) client.close();

	char[MAX_PACKET_SIZE] packet = void;
	char[] all;

Lreceive:
	while(1)
	{
	    const len = client.receive(packet);
	    if (len == 0)
	    	break;
	    else if (len < 0)
	    	return;
	    all ~= packet;
	    for (int p=0; p<all.length-3; ++p)
	    	if (all[p..p+4] == "\r\n\r\n")
	    	{
	    		//got headers
				client.shutdown(SocketShutdown.RECEIVE);
	    		break Lreceive;
	    	}
	}

	scope(exit) client.shutdown(SocketShutdown.BOTH);

    enum HEADERS =
    		"HTTP/1.1 200 OK\r\n"
			"Connection: close\r\n"
			"Cache-Control: no-cache\r\n"
			"Pragma: no-cache\r\n"
			"Content-Type: application/json\r\n"
			"Access-Control-Allow-Origin: *\r\n"
			"Access-Control-Allow-Method: GET,OPTIONS\r\n"
	        "Access-Control-Allow-Headers: cache-control,x-date,authorization,content-type\r\n\r\n";

    if (all[0..9] == "OPTIONS /")
    {
		client.send(HEADERS);
    }
	else if (all[0..17] == "GET /json HTTP/1.")
	{
		client.send(HEADERS ~ `[{"version":1},`);
		client.send(log);
		client.send(`]`);
	}
	else if (all[0..19] == "GET /toggle HTTP/1.")
	{
    	auto sock = new UdpSocket(AddressFamily.INET);
    	sock.sendTo("toggle", cast(Address)lastSender);
		client.send(HEADERS ~ "OK");
	}
	else
	{
		client.send("HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\nNot Found.");
	}
}


void main()
{
	spawn(&payloadhandler_udp);
	spawn(&payloadhandler_tcp);
    writeln("Listening for TOILET on port ", TOILET_PORT);

    auto sock = new TcpSocket(AddressFamily.INET);
    sock.bind(new InternetAddress(HTTP_PORT));
    scope(exit) sock.close();
    sock.listen(5);
    writeln("Listening for HTTP on port ", HTTP_PORT);
    while (1)
    {
    	auto client = sock.accept();
    	debug writeln("HTTP client connected from ", client.remoteAddress);
    	spawn(&handle_http_client, cast(shared)client);
    }
}

