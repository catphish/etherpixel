require 'socket'
udp = UDPSocket.new
loop do
  data = []
  180.times do
    data << rand(256)
  end
  udp.send(data.pack('C*'), 0, '192.168.88.5', 8888)
  sleep 0.01
end
