# Robocup_Avances

Este repositorio reúne **dos líneas de trabajo**:

1. **Avances de la Robocup anterior** (histórico consolidado para trazabilidad).
2. **Nuevos avances** realizados para la iteración actual.

El objetivo es que cualquier persona (incluyendo colaboradores en otros países) pueda **replicar el workspace completo** con el menor número de pasos posible.

> Nota: Este repo incluye la carpeta `ros2_ws` completa, con **todas** sus subcarpetas y artefactos (incluyendo `build/`, `install/` y `log/`) para facilitar la reproducción. Si prefieres compilar desde cero, encontrarás abajo los comandos recomendados.

---

## Entorno de referencia

- **SO base**: Ubuntu 22.04 LTS  
- **ROS 2**: Humble 
- **Herramientas**: `colcon`, `rosdep`, `git`, `gcc/g++`, `python3-pip` (según paquetes)

> Aunque el repo trae artefactos de compilación, **se recomienda** generar una compilación limpia en tu equipo para evitar inconsistencias de plataforma o de toolchain.

---

## Estructura del repositorio

```
Robocup_Avances/
└─ ros2_ws/
   ├─ src/                 # Paquetes fuente (históricos + nuevos avances)
   ├─ build/               # Artefactos de compilación (incluidos para referencia)
   ├─ install/             # Overlay instalable (incluido)
   └─ log/                 # Registros de compilación y ejecución
```

- Los **paquetes históricos** se mantienen para consulta, comparación y reuso.
- Los **paquetes nuevos** incorporan mejoras de arquitectura, nodos, mensajes/servicios, y/o integración con hardware/simulación, según la línea de trabajo actual.

---

## Clonado del repositorio

```bash
git clone https://github.com/Julinzzz/Robocup_Avances.git
cd Robocup_Avances/ros2_ws
```

Si utilizas Git LFS (para archivos grandes, modelos, bags, etc.), asegúrate de tenerlo instalado previamente:

```bash
git lfs install
```

---

## Instalación de dependencias

Desde la raíz del workspace:

```bash
cd ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

> Si algún paquete requiere dependencias del sistema o de Python específicas, revísalas en el `package.xml` o en el `README` interno de cada paquete.

---

## Compilación

### Opción A: Compilación limpia (recomendado)
Elimina artefactos previos y compila:

```bash
rm -rf build/ install/ log/
colcon build --merge-install
```

### Opción B: Usar artefactos incluidos
Puedes intentar usar el overlay existente:

```bash
source install/setup.bash
```

Si el entorno difiere (OS/compilador/ROS 2), es probable que necesites la **Opción A**.

---

## Ejecución

Ejemplo genérico (ajusta a tus paquetes/nodos reales):

```bash
# En una terminal
cd ros2_ws
source install/setup.bash
ros2 launch <paquete_lanzador> <archivo.launch.py>

# o lanzar nodos sueltos:
ros2 run <paquete> <nodo_ejecutable> [--args]
```

### Configuración útil
- **ROS_DOMAIN_ID** para evitar colisiones en redes compartidas:
  ```bash
  export ROS_DOMAIN_ID=42
  ```
- **RMW** (si usas otra implementación DDS):
  ```bash
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # o rmw_cyclonedds_cpp, etc.
  ```

---

## Qué incluye cada bloque de avances

- **Avances Robocup (histórico)**  
  - Nodos, launch files y configuraciones usados en la edición anterior.  
  - Código y scripts auxiliares (simulación, pruebas, utilidades).

- **Nuevos avances**  
  - Ajustes de arquitectura.  
  - Optimización de nodos/algoritmos.  
  - Integración adicional (sensado/actuación/simulación).  
  - Mejoras en pipelines de ejecución y pruebas.

> Si requieres un **mapa detallado por paquete** (descripcion, dependencias, punto de entrada, ejemplos de uso), dímelo y genero una tabla por cada paquete en `src/`.

---

## Buenas prácticas sugeridas

- Trabajar con **ramas por feature** y PRs descriptivos.  
- Mantener **launch files** con parámetros documentados.  
- Incluir **tests** (`ament_cmake_gtest`/`ament_cmake_pytest`) cuando aplique.  
- Documentar **interfaz de mensajes/servicios** cuando se cambie un `.msg`/`.srv`.  

---

## Problemas conocidos y solución

- **Incompatibilidad de artefactos**: si ves errores tipo ABI/soname o símbolos no resueltos, borra `build/ install/ log/` y compila limpio.  
- **Dependencias faltantes**: ejecuta `rosdep install ...` como se indica arriba.  
- **Descubrimiento DDS en red**: ajustar `ROS_DOMAIN_ID` o la implementación de RMW puede resolver conflictos.

---

## Contribuciones

1. Crea una rama: `git checkout -b feature/mi_cambio`.
2. Confirma cambios: `git commit -m "Descripción clara del cambio"`.
3. Empuja la rama: `git push origin feature/mi_cambio`.
4. Abre un **Pull Request**.
