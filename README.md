# xlgyrod
Daemon for xlgyro

Do:

mkdir build && cd build && cmake .. && make

Teset Option(Simulate):


./xlgyrod --test=virutal interface to which you want send data
  
  
  Example: ./xlgyrod --tty=/dev/pts/1 --test=/dev/pts/2
  
  
  Creating vitual interfaces: socat -d -d pty pty
