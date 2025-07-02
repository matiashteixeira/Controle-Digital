import serial
import time

porta = 'COM4'        # Altere para sua porta
baudrate = 115200
arquivo_saida = 'dados.txt'
caractere_inicial = 'A'

with serial.Serial(porta, baudrate, timeout=1) as ser, open(arquivo_saida, 'w') as f:
    time.sleep(2)
    ser.write(caractere_inicial.encode())
    print(f"Caractere '{caractere_inicial}' enviado. Iniciando leitura... Pressione Ctrl+C para parar.")

    try:
        while True:
            linha = ser.readline().decode(errors='ignore').strip()
            if linha:
                print(linha)
                f.write(linha + '\n')
    except KeyboardInterrupt:
        print("\nGravação encerrada.")
