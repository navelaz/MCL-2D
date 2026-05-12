# Localización Monte Carlo (Filtro de Partículas) 🌸

Este proyecto consiste en la implementación de un sistema de localización para robots móviles utilizando el método de **Monte Carlo**, desarrollado en **Python** y simulado en **CoppeliaSim**.

## 🚀 Entornos Utilizados
Se trabajó con dos escenas distintas para validar el comportamiento del filtro:
1. **Entorno Simétrico:** Un mapa con geometrías repetitivas.
2. **Entorno Asimétrico:** Un mapa con puntos de referencia únicos.

## 📺 Videos de Demostración
* 🎥 [Video Entorno Simétrico](LINK_AQUI)
* 🎥 [Video Entorno Asimétrico](LINK_AQUI)

## 🛠️ Tecnologías
* **Lenguaje:** Python
* **Simulador:** CoppeliaSim (V-REP)
* **Bibliotecas:** NumPy, OpenCV, ZMQ (Remote API)

> [!IMPORTANT]
> **Nota de inicialización:** En el entorno simétrico, debido a la similitud de las paredes, a veces el filtro no converge correctamente en el primer intento. Si la nube de partículas no se agrupa en la posición real del robot, presiona `Ctrl+C` y reinicia el script de Python.

---
*Proyecto de Robótica - Implementación de Filtro de Partículas*
