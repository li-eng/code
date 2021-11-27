import time
import math
import smbus
import matplotlib.pyplot as plt


# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards
address = 0x68       # This is the address value read via the i2cdetect command

# Now wake the 6050 up as it starts in sleep mode
bus.write_byte_data(address, power_mgmt_1, 0)

agx=0
agy=0
agz=0

lastTime = 0
t=0

acceRatio = 16384.0
gyroRatio = 131.0

n_sample = 8

aaxs = [0,0,0,0,0,0,0,0]
aays = [0,0,0,0,0,0,0,0]
aazs = [0,0,0,0,0,0,0,0]

a_x = [0,0,0,0,0,0,0,0,0,0]
a_y = [0,0,0,0,0,0,0,0,0,0]
a_z = [0,0,0,0,0,0,0,0,0,0]

Px = 1
Py = 1
Pz = 1

def read_byte(adr):
    return bus.read_byte_data(address, adr)

def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr + 1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

class Setup():
    def __init__(self):
        self.time = 200
        axo = 0
        ayo = 0
        azo = 0
        gxo = 0
        gyo = 0
        gzo = 0

        for i in range(0,self.time):
            gx = read_word_2c(0x43)
            gy = read_word_2c(0x45)
            gz = read_word_2c(0x47)

            ax = read_word_2c(0x3b)
            ay = read_word_2c(0x3b)
            az = read_word_2c(0x3b)

            axo += ax
            ayo += ay
            azo += az

            gxo += gx
            gyo += gy
            gzo += gz

        axo /= self.time
        ayo /= self.time
        azo /= self.time
        self.axo = axo
        self.ayo = ayo
        self.azo = azo

        gxo /= self.time
        gyo /= self.time
        gzo /= self.time
        self.gxo = gxo
        self.gyo = gyo
        self.gzo = gzo

setup = Setup()

#display data
fig = plt.figure()
ax = fig.add_subplot(1,2,1)
ay = fig.add_subplot(1,2,2)

ax.set_xlabel('Time')
ax.set_ylabel('xrotation')
ax.set_title('')

ay.set_xlabel('Time')
ay.set_ylabel('yrotation')

line = None
plt.grid(True)
plt.ion()
obsX = []
obsY = []

obsY1 = []

while True:
    now = time.perf_counter()
    dt = now - lastTime
    lastTime = now
    
    time.sleep(0.1)
    
    gx = read_word_2c(0x43)
    gy = read_word_2c(0x45)
    gz = read_word_2c(0x47)

    ax = read_word_2c(0x3b)
    ay = read_word_2c(0x3d)
    az = read_word_2c(0x3f)

    accx = ax / acceRatio
    accy = ay / acceRatio
    accz = az / acceRatio

    aax = math.atan(accy/accz)*(180)/math.pi
    aay = math.atan(accx/accz)*(-180)/math.pi
    aaz = math.atan(1)*180/math.pi

    aax_sum = 0
    aay_sum = 0
    aaz_sum = 0

    for i in range(1,n_sample):
        aaxs.insert(i-1, i)
        aax_sum += aaxs[i]*i
        aays[i-1] = aays[i]
        aay_sum += aays[i] * i
        aazs[i - 1] = aazs[i]
        aaz_sum += aazs[i] * i

    aaxs[n_sample-1] = aax
    aax_sum += aax * n_sample
    aax = (aax_sum / (11*n_sample/2.0)) * 9 / 7.0
    aays[n_sample - 1] = aay # 此处应用实验法取得合适的系数
    aay_sum += aay * n_sample #本例系数为9 / 7
    aay = (aay_sum / (11 * n_sample / 2.0)) * 9 / 7.0
    #aazs[n_sample - 1] = aaz
    #aaz_sum += aaz * n_sample
    #aaz = (aaz_sum / (11 * n_sample / 2.0)) * 9 / 7.0

    gyrox = - (gx) / gyroRatio * dt # x轴角速度
    gyroy = - (gy) / gyroRatio * dt # y轴角速度
    gyroz = - (gz) / gyroRatio * dt # z轴角速度

    agx += gyrox                             #x轴角速度积分
    agy += gyroy                             #x轴角速度积分
    agz += gyroz

    #kalman start
    Sx = 0
    Rx = 0
    Sy = 0
    Ry = 0
    Sz = 0
    Rz = 0

    for i in range(1,10):
        a_x[i-1] = a_x[i]
        Sx += a_x[i]
        a_y[i - 1] = a_y[i]
        Sy += a_y[i]
        a_z[i - 1] = a_z[i]
        Sz += a_z[i]

    a_x[9] = aax
    Sx += aax
    Sx /= 10 # x轴加速度平均值
    a_y[9] = aay
    Sy += aay
    Sy /= 10 # y轴加速度平均值
    a_z[9] = aaz
    Sz += aaz
    Sz /= 10

    for i in range(0,10):
        Rx += math.pow(abs(a_x[i] - Sx),0.5)
        Ry += math.pow(abs(a_y[i] - Sy),0.5)
        Rz += math.pow(abs(a_z[i] - Sz),0.5)

    Rx = Rx / 9    #得到方差
    Ry = Ry / 9
    Rz = Rz / 9

    Px = Px + 0.0025                         # 0.0025在下面有说明...
    Kx = Px / (Px + Rx)                     #计算卡尔曼增益
    agx = agx + Kx * (aax - agx)             #陀螺仪角度与加速度计速度叠加
    Px = (1 - Kx) * Px

    Py = Py + 0.0025
    Ky = Py / (Py + Ry)
    agy = agy + Ky * (aay - agy)
    Py = (1 - Ky) * Py

    Pz = Pz + 0.0025
    Kz = Pz / (Pz + Rz)
    agz = agz + Kz * (aaz - agz)
    Pz = (1 - Kz) * Pz
    
    print('x:',agx)
    print('y:',agy)
    #print('z:',agz)
    
    t = t + 0.1
    obsX.append(t)
    obsY.append(agx)
    obsY1.append(agy) 
    #print('x_ro:',agx)
    #print('y_ro:',agy)
    #print(np.mean(obsY))
    
    if line is None:
        line = plt.plot(obsX, obsY, '-g', marker = '*')[0]
        line1 = plt.plot(obsX, obsY1, '-g', marker = '*')[0]
    
    line.set_xdata(obsX)
    line.set_ydata(obsY)
    
    line1.set_xdata(obsX)
    line1.set_ydata(obsY1)    

    
    #plt.set_xlim([t-20,t+10])
    #plt.set_ylim([-2,2])
    
    #line1.set_xlim([t-20,t+10])
    #line1.set_ylim([-2,2])
       
    
    plt.pause(0.1)