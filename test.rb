require 'socket'
udp = UDPSocket.new
loop do
  16.times do |n|
    data = []
    data << n
    data << ((n == 15) ? 1 : 0)
    180.times do
      data << rand(256)
    end
    udp.send(data.pack('C*'), 0, '192.168.88.5', 8888)
  end
  sleep 0.01
end
