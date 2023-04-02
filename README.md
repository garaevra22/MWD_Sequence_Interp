# MWD_Sequence_Interp

 Interpreter of control sequences for MWD. 
 Предназначен для системы на базе отечественного микроконтроллера (МК) K1986BE92QI (MILANDR). 
 Автор - Р.А.Гараев: garaevra@mail.ru. 
 Компилятор:  arm-none-eabi-gcc.exe (ARM GCC Compiler из IDE Em::Blocks 2.30).
 
Представлены 2 файла из большого проекта управляющего ПО внутрискважинного микропроцессорного модуля телеметрии (далее модуля телесистемы) для системы наклонно-направленого бурения с передачей данных на поверхность по гидроканалу (measurement while drilling - MWD). Модуль телесистемы взаимодействует с модулями инклинометра,  измерения гамма излучения и др. Состав и порядок передаваемых параметров задается управляющими последовательностями, широко применяемыми в подобных системах (см., например, 6.	Kripa Nidhi, David Erdos. Advanced Mud Pulse Telemetry: M-Ary Encoding for MWD Tools [Chapter 2]. [Электронный ресурс]. URL:https://www.erdosmiller.com/advanced-mud-pulse-telemetry-mary ). Синтаксис последовательностей поддерживает объединение в блоки параметров с возможностью циклического повторения заданное количество раз. 
В файле Interp.c представлен код одной задачи (потока) FreeRTOS, который интерпретирует управляющую последовательность и заполняет небольшую дополнительную очередь задачи (потока), занимающейся сбором данных по протоколу modbus от инклинометра, гамма-модуля и пр., а затем передающей нужные параметры по гидроканалу. При заполнении этой очереди, поток интерпретатор блокируется до появления свободного места в очереди. 

Выбор каждого очередного параметра в потоке передачи по гидроканалу производится чтением очередного элемента (сообщения) из очереди. Освобождение хотя бы одного места в очереди вызывает разблокировку задачи интерпретатора и загрузку в очередь очередного элемента в соответствии с интерпретируемой последовательностью. 
Файл User_defsM.h содержит определение ряда типов и параметров, используемых в Interp.c. 
