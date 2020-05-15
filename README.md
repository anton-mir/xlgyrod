# xlgyrod
Daemon for xlgyro

Do:

mkdir build && cd build && cmake .. && make

Teset Option(Simulate):
### Carefully! Not running socat and incorrect PTY can cause xlgyrod to malfunction.

### 1 Run socat 
Open new terminal tab and run:
socat -d -d pty pty

### 2 Run xlgyrod test 
1. Take PTY from socat
2. Open new terminal tab and run:
./xlgyrod --tty=PTY(output) --test=PTY(input)  
Input and output can be swapped.

Example: ./xlgyrod --tty=/dev/pts/1 --test=/dev/pts/2
  
  
