## 25.27 - Sistemas Embebidos TRABAJO PRÁCTICO N° 1

## MANUAL DE USUARIO 

# SISTEMA DE USUARIOS 

**El sistema emplea un display en el cual se presentan los datos ingresados o los cuales se están configurando, un encoder con botón para ingresar al menú y los datos necesarios; y un set de Leds los cuales emulan el pestillo de ingreso o una alarma de error.**

<p align="center">
	<img src="image/README/board.png" alt="board" width="600"/>
</p>

---

### **Menú de inicio**

La pestaña inicial presenta una serie de alternativas de uso las cuales se catalogan de la siguiente manera:
- Add User (ADD ): Esta opción permite agregar un usuario al sistema, para ello requiere ingresar un ID y una contraseña.
- Delete User (dELE): Esta opción permite eliminar un usuario al sistema, para ello requiere ingresar un ID y una contraseña, los cuales ambos deben ser válidos y pertenecer a un usuario existente.
- Change Password (CHnG): Esta opción permite cambiar una contraseña de un usuario, para ello requiere ingresar un ID existente,  su contraseña actual válida y la nueva contraseña.
- Access (ACCS): Brinda el acceso a un usuario existente en el sistema. Requiere que el ID exista y la contraseña sea la correspondiente.
- Change Brightness (brIG): Permite cambiar el nivel de brillo del display de 0 a 25.

### **Uso del Encoder**

Para ingresar los datos se utiliza un encoder, el cual permite girar a la izquiera, la derecha y pulsar. A continuación se comentan los usos posibles:
- Derecha/Izquiera: Esto permite cambiar la selección actual del menú o cambiar el dígito actual ingresado, ya sea para ID o contraseña.
- Pulsar 1 vez: Esto ingresa a la opción seleccionada. En caso de estar en el menú ingresa a lo que se desea y en caso de realizarse modificando un dígito, confirma la selección.
- Pulsar 2 veces: Esto borra la última selección. Es utilizado para correrir errores de ingreso de datos. Requiere de realizarse en un lapso de tiempo o se considera un único click.
- Mantener pulsado: Esta opción permite retornar al menú, cancelando el ingreso de datos actuales. En caso de realizarse en el menú de cambio de brillo, se setea el máximo valor posible.

En todo caso de utilizar el botón/pulsador, se prende un LED de la placa del sistema.

### **Uso de LEDs**

Hay 3 LEDs en el dispositivo actual, en caso de que los tres estén prendidos en simultáneo, corresponde a la habilitación del pestillo. En caso en el que solo esté prendido el del medio, esto equivale a accionar una alerta de error de acceso.

