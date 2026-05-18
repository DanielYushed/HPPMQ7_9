import os
import shutil
import subprocess
import matplotlib.pyplot as plt

DIR_OBS = "inputs_obstaculos"
DIR_PEND = "inputs_pendientes"
DIR_EXE = "output"
DIR_IMG = "resultados_img"

TEMP_PEND = "pendientes.txt"
TEMP_OBS = "obstaculos1.txt"
TEMP_TRAY = "tray.txt"

def preparar_entorno_base():
    "Crea las carpetas principales si no existen."
    for carpeta in [DIR_OBS, DIR_PEND, DIR_EXE, DIR_IMG]:
        if not os.path.exists(carpeta):
            os.makedirs(carpeta)

def leer_pendientes(ruta_archivo):
    "Extrae el inicio y la meta para dibujarlos en la gráfica."
    with open(ruta_archivo, 'r') as f:
        datos = f.read().split()
        if len(datos) >= 8:
            inicio = (float(datos[4]), float(datos[5]))
            meta = (float(datos[6]), float(datos[7]))
            return inicio, meta
    return (0,0), (1,1)

def leer_obstaculos(ruta_archivo):
    "Extrae los obstáculos para dibujarlos como círculos."
    obstaculos = []
    try:
        with open(ruta_archivo, 'r') as f:
            for linea in f:
                datos = linea.split()
                if len(datos) == 4:
                    obstaculos.append((float(datos[0]), float(datos[1]), float(datos[2])))
    except FileNotFoundError:
        pass
    return obstaculos

def graficar_y_guardar(ruta_imagen, inicio, meta, obstaculos, nombre_exe, nombre_obs, nombre_pend):
    "Lee tray.txt y genera un mapa 2D del recorrido."
    x_tray = []
    y_tray = []
    
    try:
        with open(TEMP_TRAY, 'r') as f:
            for linea in f:
                datos = linea.split(',')
                if len(datos) == 1: 
                    datos = linea.split()
                if len(datos) >= 2:
                    x_tray.append(float(datos[0]))
                    y_tray.append(float(datos[1]))
    except FileNotFoundError:
        print(f" No se generó tray.txt para {nombre_exe}")
        return

    fig, ax = plt.subplots(figsize=(8, 8))
    
    for (ox, oy, radio) in obstaculos:
        circulo = plt.Circle((ox, oy), radio, color='red', alpha=0.3)
        ax.add_patch(circulo)
        ax.plot(ox, oy, 'rx', markersize=4)

    ax.plot(x_tray, y_tray, 'b-', linewidth=2, label='Trayectoria')
    ax.plot(inicio[0], inicio[1], 'go', markersize=8, label='Inicio')
    ax.plot(meta[0], meta[1], 'k*', markersize=10, label='Meta')

    ax.set_aspect('equal', adjustable='box')
    ax.set_title(f"EXE: {nombre_exe}\nMapa: {nombre_obs} | Pendientes: {nombre_pend}")
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend()

    plt.savefig(ruta_imagen, dpi=150, bbox_inches='tight')
    plt.close()

def main():
    preparar_entorno_base()
    
    archivos_exe = [f for f in os.listdir(DIR_EXE) if f.endswith('.exe')]
    archivos_obs = [f for f in os.listdir(DIR_OBS) if f.endswith('.txt')]
    archivos_pend = [f for f in os.listdir(DIR_PEND) if f.endswith('.txt')]

    if not archivos_exe:
        print(f"No se encontraron ejecutables en '{DIR_EXE}/'.")
        return
    if not archivos_obs or not archivos_pend:
        print("Faltan archivos de prueba en las carpetas de inputs.")
        return

    print("Iniciando orquestación de pruebas...")
    
    for exe in archivos_exe:
        ruta_exe = os.path.join(DIR_EXE, exe)
        nombre_exe_sin_ext = exe.replace('.exe', '')
        
        carpeta_salida_exe = os.path.join(DIR_IMG, nombre_exe_sin_ext)
        if not os.path.exists(carpeta_salida_exe):
            os.makedirs(carpeta_salida_exe)
            
        print(f"\n>> Evaluando código: {exe}")
        
        for pend in archivos_pend:
            ruta_pend = os.path.join(DIR_PEND, pend)
            
            for obs in archivos_obs:
                ruta_obs = os.path.join(DIR_OBS, obs)
                
                print(f"   -> Mapa: {obs} | Config: {pend}")
                
                shutil.copy(ruta_pend, TEMP_PEND)
                shutil.copy(ruta_obs, TEMP_OBS)
                if os.path.exists(TEMP_TRAY):
                    os.remove(TEMP_TRAY)
                
                try:
                    subprocess.run([os.path.abspath(ruta_exe)], cwd=os.getcwd(), capture_output=True, text=True, timeout=10)
                except subprocess.TimeoutExpired:
                    print(f"      [!] Timeout: Bucle infinito detectado.")
                except Exception as e:
                    print(f"      [!] Error de ejecución: {e}")
                
                inicio, meta = leer_pendientes(TEMP_PEND)
                lista_obstaculos = leer_obstaculos(TEMP_OBS)
                
                nombre_salida = f"{obs.replace('.txt','')}_{pend.replace('.txt','')}.png"
                ruta_imagen = os.path.join(carpeta_salida_exe, nombre_salida)
                
                graficar_y_guardar(ruta_imagen, inicio, meta, lista_obstaculos, nombre_exe_sin_ext, obs, pend)

    for temp_file in [TEMP_PEND, TEMP_OBS, TEMP_TRAY]:
        if os.path.exists(temp_file):
            os.remove(temp_file)
            
    print(f"\n¡Listo! Resultados organizados en subcarpetas dentro de '{DIR_IMG}/'.")

if __name__ == "__main__":
    main() 