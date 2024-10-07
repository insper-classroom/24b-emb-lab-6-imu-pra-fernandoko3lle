import serial
import uinput

ser = serial.Serial('/dev/ttyACM0', 115200)

# Create new mouse device
device = uinput.Device([
    uinput.BTN_LEFT,
    uinput.BTN_RIGHT,
    uinput.REL_X,
    uinput.REL_Y,
])

# Limiar de velocidade para ativar o clique (ajuste conforme necessário)
VELOCIDADE_LIMIAR = 20

def parse_data(data):
    axis = data[0]  # 0 para X, 1 para Y
    value = int.from_bytes(data[1:3], byteorder='big', signed=True)
    if axis == 0:
        vel = data[3]  # Velocidade extraída
    else:
        vel = 0
    print(f"Received data: {data}")
    print(f"axis: {axis}, value: {value}, vel: {vel}")
    return axis, value, vel

def move_mouse(axis, value):
    if axis == 0:    # Eixo X
        device.emit(uinput.REL_X, value)
    elif axis == 1:  # Eixo Y
        device.emit(uinput.REL_Y, value)

def check_for_click(vel):
    # Se a velocidade exceder o limite, executa um clique
    if vel > VELOCIDADE_LIMIAR:
        print('Click detected! Performing mouse click.')
        device.emit(uinput.BTN_LEFT, 1)  # Pressionar botão esquerdo
        device.emit(uinput.BTN_LEFT, 0)  # Soltar botão esquerdo

try:
    # Sincronizando o pacote
    while True:
        print('Esperando pelo pacote de sincronização...')
        while True:
            data = ser.read(1)
            if data == b'\xff':
                break
            else:
                print(f"Erro recebido: {data}")

        # Ler 4 bytes da UART
        data = ser.read(4)  # Agora estamos lendo 4 bytes, incluindo a velocidade
        axis, value, vel = parse_data(data)
        move_mouse(axis, value)
        check_for_click(vel)  # Verifica se é necessário fazer o clique

except KeyboardInterrupt:
    print("Programa interrompido pelo usuário.")
except Exception as e:
    print(f"Ocorreu um erro: {e}")
finally:
    ser.close()
