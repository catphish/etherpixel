require 'socket'
udp = UDPSocket.new
loop do
  udp.send('hello', 0, '192.168.88.5', 8888)
  sleep 0.1
end
