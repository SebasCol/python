import cv2 
import numpy as np
from tkinter import *
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D

import xlwt

import sys
import xlrd
import os
from tkinter import messagebox
from tkinter import ttk

style0 = xlwt.easyxf('''font: name Times New Roman, color-index white, bold on;pattern:pattern solid, fore_colour red''',num_format_str='#,##0.00')

#FUNCION PARA GUARDAR PREDICCION EN DOCUMENTO PLANO
def guardar_prediccion(vector):
    
    #BUSQUEDA LISTADO DE ARCHIVOS DEL DIRECTORIO DE LAS PRUEBAS
    contenido = os.listdir('Data/Documentos')
    
    #ESPECIFICACIONES INICIALES VENTANA 
    Vguardar=Tk()
    Vguardar.resizable(0,0)
    Vguardar.geometry("270x150+%d+50" % ((Vguardar.winfo_screenwidth()-350)/2))
    Vguardar.title("Ventana guardar")

    #FUNCION PARA BOTON "GUARDAR"
    def guarda():
        try:
            #CREACION DEL DOCUMENTO DONDE QUEDARA GUARDADO LOS DATOS DE LA PRUEBA
            archivo = open('Data/Documentos/Prueba'+str(len(contenido)+1)+'.csv','w')

            #ESCRITURA EN EL DOCUMENTO CSV
            for j in range(len(vector)):
                for i in range(len(globals()["tamaño"+str(j)])):
                    archivo.write(str(globals()["medidasx"+str(j)][i]))
                    archivo.write(',')
                archivo.write(str(vector[j][6])+'\n')
                for i in range(len(globals()["tamaño"+str(j)])):
                    archivo.write(str(globals()["medidasy"+str(j)][i]))
                    archivo.write(',')
                for i in range(len(globals()["tamaño"+str(j)])):
                    archivo.write(str(globals()["medidasy"+str(j)][i]))
                    archivo.write(',')
                archivo.write(str(vector[j][6])+'\n')
                for i in range(len(globals()["tamaño"+str(j)])):
                    archivo.write(str(globals()["prediccionx"+str(j)][i]))
                    archivo.write(',')
                archivo.write(str(vector[j][6])+'\n')
                for i in range(len(globals()["tamaño"+str(j)])):
                    archivo.write(str(globals()["predicciony"+str(j)][i]))
                    archivo.write(',')
                archivo.write(str(vector[j][6])+'\n')
                for i in range(len(globals()["tamaño"+str(j)])):
                    archivo.write(str(globals()["tamaño"+str(j)][i]))
                    archivo.write(',')
                archivo.write(str(vector[j][6])+'\n')
##################################################                
                for i in range(len(globals()["medidasz"+str(j)])):
                    archivo.write(str(globals()["medidasz"+str(j)][i]))
                    archivo.write(',')
                archivo.write(str(vector[j][6])+'\n')
                for i in range(len(globals()["prediccionz"+str(j)])):
                    archivo.write(str(globals()["prediccionz"+str(j)][i]))
                    archivo.write(',')
                archivo.write(str(vector[j][6])+'\n')
##########################################################                
            archivo.close()

            #VENTANA ADVERTENCIA DE GUARDADO EXITOSO
            messagebox.showinfo("Advertencia","Guardo correctamente")

            #DESTRUCCION VENTANA 
            Vguardar.destroy()
        except:
            
            #VENTANA ADVERTENCIA DE ERROR AL GUARDAR
            messagebox.showinfo("Advertencia","Error al guardar")

            #DESTRUCCION VENTANA 
            Vguardar.destroy()

    #FUNCION PARA BOTON "VOLVER"
    def salida():
        
        #DESTRUCCION VENTANA 
        Vguardar.destroy()

    #TEXTO EN PANTALLA
    Lab1=Label(Vguardar,text="Desea guardar la Prueba # "+str(len(contenido)+1), fg="blue",
               font=("Yu Gothic",12,"bold"),anchor="center").place(x=25,y=20)

    #BOTONES PARA PANTALLA
    btn1=Button(Vguardar, command=guarda,  fg="blue",  text="Guardar",font=("Yu Gothic",12,"bold") ,
                anchor="center",width=10,height=1).place(x = 140, y = 70)
    btn2=Button(Vguardar, command=salida,  fg="blue",  text="Volver",font=("Yu Gothic",12,"bold") ,
                anchor="center",width=10,height=1).place(x = 20, y = 70)

    Vguardar.mainloop()

#FUNCION PARA GUARDAR UMBRAL OBJETO NUEVO
def guardar_umbral(Hm,Sm,Vm,HM,SM,VM):
    
    #ESPECIFICACIONES INICIALES VENTANA 
    Vguardar=Tk()
    Vguardar.resizable(0,0)
    Vguardar.geometry("270x330+%d+50" % ((Vguardar.winfo_screenwidth()-350)/2))
    Vguardar.title("Ventana guardar")

    #LECTURA DEL DOCUMENTO DONDE ESTA GUARDADO LOS DATOS DE LOS OBJETOS
    archivo = open('Data/Objetos/Objetos.csv','r')

    #VARIABLES PARA CONTAR OBJETOS Y GUARDAR INFORMACION
    objetos = 0
    lista = []

    #LECTURA POR LINEA DE LA INFORMACION EN EL DOCUMENTO
    while True:
        hoja = archivo.readline()
        if not hoja:
            break
        datos = list(hoja.split(','))
        lista.append(datos)
        objetos = objetos + 1  
    archivo.close()

    #TEXTO EN PANTALLA
    Lab1=Label(Vguardar,text="Desea guardar el objeto # "+str(objetos+1),
               fg="blue",  font=("Yu Gothic",12,"bold"),
               anchor="center").place(x = 25,y = 20)
    Lab2=Label(Vguardar,text="H minimo "+str(Hm), fg="blue",
               font=("Yu Gothic",12,"bold"),anchor="center").place(x = 25,y = 50)
    Lab3=Label(Vguardar,text="H maximo "+str(HM), fg="blue",
               font=("Yu Gothic",12,"bold"),anchor="center").place(x = 150,y = 50)
    Lab4=Label(Vguardar,text="S minimo "+str(Sm), fg="blue",
               font=("Yu Gothic",12,"bold"),anchor="center").place(x = 25,y = 80)
    Lab5=Label(Vguardar,text="S maximo "+str(SM), fg="blue",
               font=("Yu Gothic",12,"bold"),anchor="center").place(x = 150,y = 80)
    Lab6=Label(Vguardar,text="V minimo "+str(Vm), fg="blue",
               font=("Yu Gothic",12,"bold"),anchor="center").place(x = 25,y = 110)
    Lab7=Label(Vguardar,text="V maximo "+str(VM), fg="blue",
               font=("Yu Gothic",12,"bold"),anchor="center").place(x = 150,y = 110)
    Lab8=Label(Vguardar,text="Color "           , fg="blue",
               font=("Yu Gothic",12,"bold"),anchor="center").place(x = 25,y = 140)
    Lab9=Label(Vguardar,text="Diametro "        , fg="blue",
               font=("Yu Gothic",12,"bold"),anchor="center").place(x = 25,y = 200)

    

    #FUNCION PARA BOTON "GUARDAR"
    def guarda():

        #VARIABLE PARA GUARDAR DATO DE LOS CUADROS DE TEXTO
        colore = color.get()
        diametroe = diametro.get()

        #AGREGARNDO INFORMACION NUEVA AL VECTOR 
        numero = len(lista)
        lista.append([])
        lista[numero].append(str(Hm))
        lista[numero].append(str(Sm))
        lista[numero].append(str(Vm))
        lista[numero].append(str(HM))
        lista[numero].append(str(SM))
        lista[numero].append(str(VM))
        lista[numero].append(str(colore))
        lista[numero].append(str(diametroe))
        lista[numero].append(str("objeto"+str(objetos+1)))
        lista[numero].append(str(""))

        #ARCHIVO ABIERTO EN MODO DE ESCRITURA
        archivo = open('Data/Objetos/Objetos.csv','w')

        #ESCRITURA EN ARCHIVO
        for i in lista:
            archivo.write(str(i[0]))
            archivo.write(",")
            archivo.write(str(i[1]))
            archivo.write(",")
            archivo.write(str(i[2]))
            archivo.write(",")
            archivo.write(str(i[3]))
            archivo.write(",")
            archivo.write(str(i[4]))
            archivo.write(",")
            archivo.write(str(i[5]))
            archivo.write(",")
            archivo.write(str(i[6]))
            archivo.write(",")
            archivo.write(str(i[7]))
            archivo.write(",")
            archivo.write(str(i[8]))
            archivo.write(",")
            archivo.write(str(i[9]))
            archivo.write(",")
            archivo.write("\n")
        archivo.close()

        #DESTRUCCION VENTANA 
        Vguardar.destroy()

    #FUNCION PARA BOTON "SALIDA"
    def salida():

        #DESTRUCCION VENTANA 
        Vguardar.destroy()
        
    #VARIABLES PARA CUADROS DE TEXTO
    color = StringVar()
    diametro = StringVar()
    
    #CUADROS DE TEXTO PARA PANTALLA
    Ent1=Entry(Vguardar,textvariable=color).place(x = 25,y = 170)
    Ent2=Entry(Vguardar,textvariable=diametro).place(x = 25,y = 230)
    
    #BOTONES PARA PANTALLA
    btn1=Button(Vguardar, command=guarda,  fg="blue",  text="Guardar",
                font=("Yu Gothic",12,"bold") ,anchor="center",
                width=10,height=1).place(x = 140, y = 270)
    btn2=Button(Vguardar, command=salida,  fg="blue",  text="Volver",
                font=("Yu Gothic",12,"bold") ,anchor="center",
                width=10,height=1).place(x = 20, y = 270)

    #DESTRUCCION VENTANA
    Vguardar.mainloop()
    
def umbral():
    #FUNCION GUARDAR VALOR MINIMO H
    def MIN_H(val):
        global min_H,max_H
        min_H = val
        min_H = min(max_H-1, min_H)
        cv2.setTrackbarPos('min H', "DESLIZADORES", min_H)
        
    #FUNCION GUARDAR VALOR MAXIMO H
    def MAX_H(val):
        global min_H,max_H 
        max_H = val
        max_H = max(max_H, min_H+1)
        cv2.setTrackbarPos('max H', "DESLIZADORES", max_H)
        
    #FUNCION GUARDAR VALOR MINIMO S
    def MIN_S(val):
        global min_S,max_S
        min_S = val
        min_S = min(max_S-1, min_S)
        cv2.setTrackbarPos('min S', "DESLIZADORES", min_S)
        
    #FUNCION GUARDAR VALOR MAXIMO S
    def MAX_S(val):
        global min_S,max_S
        max_S = val
        max_S = max(max_S, min_S+1)
        cv2.setTrackbarPos('max S', "DESLIZADORES", max_S)
        
    #FUNCION GUARDAR VALOR MINIMO V
    def MIN_V(val):
        global min_V,max_V
        min_V = val
        min_V = min(max_V-1, min_V)
        cv2.setTrackbarPos('min V', "DESLIZADORES", min_V)
        
    #FUNCION GUARDAR VALOR MAXIMO V
    def MAX_V(val):
        global min_V,max_V
        max_V = val
        max_V = max(max_V, min_V+1)
        cv2.setTrackbarPos('max V', "DESLIZADORES", max_V)
        
    #FUNCION CREACION DESLIZADORES
    def inicio():
        global min_H,max_H,min_S,max_S,min_V,max_V
        min_H = 0
        min_S = 0
        min_V = 0
        max_H = 180
        max_S = 255
        max_V = 255
        cv2.createTrackbar('min H', "DESLIZADORES" , min_H, 180, MIN_H)
        cv2.createTrackbar('max H', "DESLIZADORES" , max_H, 180, MAX_H)
        cv2.createTrackbar('min S', "DESLIZADORES" , min_S, 255, MIN_S)
        cv2.createTrackbar('max S', "DESLIZADORES" , max_S, 255, MAX_S)
        cv2.createTrackbar('min V', "DESLIZADORES" , min_V, 255, MIN_V)
        cv2.createTrackbar('max V', "DESLIZADORES" , max_V, 255, MAX_V)

    #TOMA DE CAPTURAS CAMARA WEB
    cap = cv2.VideoCapture(0)

    #CREACION VENTANA DESLIZADORES
    cv2.namedWindow("DESLIZADORES")

    #LLAMADO FUNCION CREACION DESLIZADORES
    inicio()

    #INICIO BUCLE WHILE 
    while True:

        #LECTURA CAPTURAS CAMARA WEB
        ret, frame = cap.read()

        #CONFIRMACION DE DATO NO NULO
        if frame is None:
            break

        
        frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_Range = cv2.inRange(frame_HSV, (min_H, min_S, min_V), (max_H, max_S, max_V))
        cv2.imshow("VIDEO_ONLINE",frame_Range)
        key = cv2.waitKey(30)
        if key == ord('q') or key == 27:
            guardar_umbral(min_H, min_S, min_V, max_H, max_S, max_V)
            cv2.destroyAllWindows()
            break

def prediccion():
    global medidasx,medidasy,prediccionx,predicciony,tamaño

    Vprediccion=Tk()
    Vprediccion.resizable(0,0)
    Vprediccion.geometry("500x500+%d+50" % ((Vprediccion.winfo_screenwidth()-350)/2))
    Vprediccion.title("Ventana prediccion")
    archivo = open('Data/Objetos/Objetos.csv','r')
    objetos = 0
    lista = []
    while True:
        hoja = archivo.readline()
        if not hoja:
            break
        datos = list(hoja.split(','))
        lista.append(datos)
        objetos = objetos + 1  
    archivo.close()
    posicion = 30
    for i in range(len(lista)):
        globals()["CV"+str(i)] = IntVar()
        
        Lab1=Label(Vprediccion,text=lista[i][8] + "  color :  "+
                   lista[i][6]+ "  diametro :  "+ lista[i][7], fg="blue",
                   font=("Yu Gothic",15,"bold"),
                   anchor="center").place(x = 10, y = 70 + posicion)
        Che1 = Checkbutton(Vprediccion, variable = globals()["CV"+str(i)],
                           onvalue = 1, offvalue = 0).place(x = 450, y = 75 + posicion)
        posicion = posicion + 30
        total_check = i+1
    def seleccion():
        Vprediccion.destroy()
        vector_objetos = []
        for i in range(total_check):
            globals()["seleccion"+str(i)] = globals()["CV"+str(i)].get()
##            print(globals()["seleccion"+str(i)],i)
            if globals()["seleccion"+str(i)] == 1:
                vector_objetos.append(lista[i])
         
        for i in range(len(vector_objetos)):
            globals()["min_H"+str(i)] = vector_objetos[i][0]
            globals()["min_S"+str(i)] = vector_objetos[i][1]
            globals()["min_V"+str(i)] = vector_objetos[i][2]
            globals()["max_H"+str(i)] = vector_objetos[i][3]
            globals()["max_S"+str(i)] = vector_objetos[i][4]
            globals()["max_V"+str(i)] = vector_objetos[i][5]
            globals()["Diametro"+str(i)] = int(vector_objetos[i][7])

            globals()["medidasx"+str(i)] = []
            globals()["medidasy"+str(i)] = []
            globals()["prediccionx"+str(i)] = []
            globals()["predicciony"+str(i)] = []

######################################            
            globals()["medidasz"+str(i)] = []
            globals()["prediccionz"+str(i)] = []
############################################
            
            globals()["tamaño"+str(i)] = []
            globals()["contador"+str(i)] = 0
##############################################
            globals()["kalmanz"+str(i)] = cv2.KalmanFilter(4,2)
            globals()["kalmanz"+str(i)].measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]],np.float32)
            globals()["kalmanz"+str(i)].transitionMatrix = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]],np.float32)
            globals()["kalmanz"+str(i)].processNoiseCov = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],np.float32) * 0.03
########################################################            
            
            #VARIABLES Y VECTORES VACIOS PARA GRAFICAR
            globals()["kalman"+str(i)] = cv2.KalmanFilter(4,2)
            globals()["kalman"+str(i)].measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]],np.float32)
            globals()["kalman"+str(i)].transitionMatrix = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]],np.float32)
            globals()["kalman"+str(i)].processNoiseCov = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],np.float32) * 0.03

        contenido = os.listdir('Data/Documentos')
        
        #TOMA DE CAPTURAS CAMARA WEB
        cap = cv2.VideoCapture(0)
        #Guardar video
        os.makedirs('Data/imagenes/PRUEBA'+str(len(contenido)+1), exist_ok=True)
        salida = cv2.VideoWriter("Data/imagenes/PRUEBA"+str(len(contenido)+1)+"/videoSalida.mp4",cv2.VideoWriter_fourcc(*'XVID'),20.0,(640,480))

        contad = 0
        inicio_de_tiempo = time.time()
    
        while True:
            
            #LECTURA CAPTURAS CAMARA WEB
            ret, frame = cap.read()

            #CONFIRMACION DE DATO NO NULO
            if frame is None:
                break
            
            #CONVERSION IMAGEN DE RGB A HSV
            frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            for i in range(len(vector_objetos)):

                colmin = (int(globals()["min_H"+str(i)]), int(globals()["min_S"+str(i)]), int(globals()["min_V"+str(i)]))
                colmax = (int(globals()["max_H"+str(i)]), int(globals()["max_S"+str(i)]), int(globals()["max_V"+str(i)]))
                                  
            #APLICACION IN RANGE PARA LA SEGMENTACION
                globals()["frame_Range"+str(i)] = cv2.inRange(frame_HSV, colmin, colmax)
                
            #FILTRADO DE IMAGEN
            #ELEMENTO ESTRUCTURANTE O NUCLEO (KERNEL)
                globals()["kernel"+str(i)] = np.ones((3,3),np.uint8)
                
            #APLICACION APERTURA (VARIACION MORFOLOGICA)
                globals()["Filtro_Apertura"+str(i)] = cv2.morphologyEx(globals()["frame_Range"+str(i)],
                                                                       cv2.MORPH_OPEN,globals()["kernel"+str(i)])
                
            #APLICACION APERTURA DE NUEVO (VARIACION MORFOLOGICA)
                globals()["Filtro_Apertura_II"+str(i)] = cv2.morphologyEx(globals()["Filtro_Apertura"+str(i)],
                                                                          cv2.MORPH_OPEN,globals()["kernel"+str(i)])

            #APLICACION DEFICION DE CONTORNOS
                globals()["Contorno"+str(i)] , _ = cv2.findContours(globals()["Filtro_Apertura_II"+str(i)] ,
                                                                    cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

            #COMPARACION PARA TOMAR SOLO DATO DEL CONTORNO MAYOR
                globals()["Comparador"+str(i)] = 0
                globals()["Contorno_Mayor"+str(i)] = []

                for j in range (len(globals()["Contorno"+str(i)])):
                    if len(globals()["Contorno"+str(i)][j]) > globals()["Comparador"+str(i)]:
                        globals()["Comparador"+str(i)] = len(globals()["Contorno"+str(i)][j])
                        globals()["Contorno_Mayor"+str(i)] = globals()["Contorno"+str(i)][j]
                globals()["Contorno"+str(i)] = globals()["Contorno_Mayor"+str(i)]
                
            #CALCULO DEL CENTROIDE A PARTIR DE LOS MOMENTOS
                if globals()["Comparador"+str(i)] != 0:
########################################################################
                    globals()["x"+str(i)],globals()["y"+str(i)],globals()["w"+str(i)],globals()["h"+str(i)]=cv2.boundingRect(globals()["Contorno"+str(i)])
                    
                    if globals()["w"+str(i)]>=globals()["h"+str(i)]:
                        diametro_max=int(globals()["w"+str(i)])
                        
                    else:
                        diametro_max=int(globals()["h"+str(i)])
                    if int(globals()["Diametro"+str(i)])<= 4:
                        coeficiente = 1
                    if int(globals()["Diametro"+str(i)])> 4 and int(globals()["Diametro"+str(i)])<= 7:
                        coeficiente = 0.55
                    if int(globals()["Diametro"+str(i)])> 7 :
                        coeficiente = 0.31
                        
                                    
                    if diametro_max>120 and diametro_max<300:
                        globals()["cz"+str(i)]=(((-0.1)*diametro_max)+(32))
                    if diametro_max>80 and diametro_max<=120:
                        globals()["cz"+str(i)]=((-0.25)*diametro_max)+(50)
                    if diametro_max>58 and diametro_max<=80:
                        globals()["cz"+str(i)]=((-5/11)*diametro_max)+(730/11)
                    if diametro_max>34 and diametro_max<=58:
                        globals()["cz"+str(i)]=((-5/6)*diametro_max)+(265/3)
                    if diametro_max>25 and diametro_max<=34:
                        globals()["cz"+str(i)]=((-10/9)*diametro_max)+(880/9)
                    if diametro_max<=25:
                        globals()["cz"+str(i)]=((-2)*diametro_max)+(120)
##                    print(globals()["cz"+str(i)],diametro_max,int(globals()["cz"+str(i)]/coeficiente ))
                    globals()["cz"+str(i)] = int(globals()["cz"+str(i)]/coeficiente)
                    
########################################################################
                    globals()["Momento"+str(i)] = cv2.moments(globals()["Contorno_Mayor"+str(i)])
                    globals()["cx"+str(i)] = int(globals()["Momento"+str(i)]['m10']/globals()["Momento"+str(i)]['m00'])
                    globals()["cy"+str(i)] = int(globals()["Momento"+str(i)]['m01']/globals()["Momento"+str(i)]['m00'])

                    cv2.circle(frame, (globals()["cx"+str(i)],globals()["cy"+str(i)]) , 5 , (0,200,255) , -1)
                    
            
                #CALCULO PREDICCION FILTRO DE KALMAN
                #MATRIX DE MEDIDA
                    globals()["medicion"+str(i)] = np.array([[np.float32(globals()["cx"+str(i)])],
                                                     [np.float32(globals()["cy"+str(i)])]])
                    globals()["kalman"+str(i)].correct(globals()["medicion"+str(i)])

                #PREDICCION DE POSICION FUTURA
                    globals()["Prediccion"+str(i)] = globals()["kalman"+str(i)].predict()
##################################################
                    #MATRIX DE MEDIDA
                    globals()["medicionz"+str(i)] = np.array([[np.float32(globals()["cx"+str(i)])],
                                                     [np.float32(globals()["cz"+str(i)])]])
                    globals()["kalmanz"+str(i)].correct(globals()["medicionz"+str(i)])

                #PREDICCION DE POSICION FUTURA
                    globals()["Prediccionz"+str(i)] = globals()["kalmanz"+str(i)].predict()



##                #MATRIX DE MEDIDA EJES X,Y
##                    medicion = np.array([[np.float32(cx),
##                                          np.float32(cy)]])
##                    kalman.correct(medicion)
##                #PREDICCION DE POSICION FUTURA EJES X,Y
##                    Prediccion = kalman.predict()
##                #DIBUJO DE LA PREDICCION EJES X,Y              
##                    cv2.circle(frame,(Prediccion[0] ,Prediccion[1]) , 5 , (200,0,255) , -1)
##                 #MATRIX DE MEDIDA EJES X,Z
##                    medicionz = np.array([[np.float32(cx),
##                                          np.float32(cz)]])
##                    kalmanz.correct(medicionz)
##                #PREDICCION DE POSICION FUTURA EJES X,Z
##                    Prediccionz = kalmanz.predict()
                









                    

                #DIBUJO DE LA PREDICCION                 
                    cv2.circle(frame,(int(globals()["Prediccion"+str(i)][0]) ,
                                      int(globals()["Prediccion"+str(i)][1])) , 5 , (200,0,255) , -1)

                #DATOS PARA GRAFICAR RESULTADOS
                    globals()["contador"+str(i)] = globals()["contador"+str(i)] + 1
                    tiempo_medio = time.time() 
                    tiempo_transcurrido1 = tiempo_medio - inicio_de_tiempo
                    globals()["tamaño"+str(i)].append(str(globals()["contador"+str(i)])+"/"+"%2f" % (tiempo_transcurrido1))
                    globals()["medidasx"+str(i)].append(globals()["cx"+str(i)])
                    globals()["medidasy"+str(i)].append(globals()["cy"+str(i)])
                    globals()["prediccionx"+str(i)].append(int(globals()["Prediccion"+str(i)][0]))
                    globals()["predicciony"+str(i)].append(int(globals()["Prediccion"+str(i)][1]))


################################################
                    globals()["medidasz"+str(i)].append(int(globals()["cz"+str(i)]))
                    globals()["prediccionz"+str(i)].append(int(globals()["Prediccionz"+str(i)][1]))
                    
##################################################
                    
                 #DIBUJO DE CONTORNOS EN LA IMAGEN
                cv2.drawContours(frame, globals()["Contorno"+str(i)], -1, (255,255,0), 3)
           
            #IMPRESION EN PANTALLA IMAGEN ORIGINAL
            cv2.imshow("VIDEO_ONLINE_ORIGINAL",frame)
            salida.write(frame)
            key = cv2.waitKey(30)
            contad += 1
            print(contad,time.time() - inicio_de_tiempo)
            contenido = os.listdir('Data/Documentos')
            cv2.imwrite("Data/imagenes/PRUEBA"+str(len(contenido)+1)+"/IMAGEN"+str(contad)+".png",frame)
            if key == ord('q') or key == 27 or contad == 100:
                tiempo_final = time.time() 
                tiempo_transcurrido = tiempo_final - inicio_de_tiempo
                print ("\nTomo %2f segundos." % (tiempo_transcurrido))
                salida.release()
                cv2.destroyAllWindows()
##                print( globals()["medidasz"+str(i)],globals()["prediccionz"+str(i)])
                guardar_prediccion(vector_objetos)
                break
    
    def select():
        vector_objetos = []
        for i in range(total_check):
            globals()["seleccion"+str(i)] = globals()["CV"+str(i)].get()
##            print(globals()["seleccion"+str(i)],i)
            if globals()["seleccion"+str(i)] == 1:
                vector_objetos.append(lista[i])
        repetido = 0
        for i in range(len(vector_objetos)):
            for j in range(len(vector_objetos)):
                if i!=j:
##                    print(vector_objetos[i][6],vector_objetos[j][6],vector_objetos[i][6]== vector_objetos[j][6])
                    if vector_objetos[i][6]== vector_objetos[j][6]:
                        repetido = 1
        if repetido == 1:
            messagebox.showinfo("Advertencia","No se puede seleccionar dos objetos del mismo color \nVuelva y seleccione solo uno")
        else:
            seleccion()
    def volver():
        Vprediccion.destroy()
    Lab1=Label(Vprediccion,text="Seleccione los objetos \na seguir ", fg="blue",
               font=("Yu Gothic",14,"bold"),anchor="center").place(x=120,y=10)
   
    btn1=Button(Vprediccion, command=select,  fg="blue",  text="Seleccionar",
                font=("Yu Gothic",10,"bold") ,anchor="center",
                width=10,height=1).place(x = 260,y = 420)
    btn2=Button(Vprediccion, command=volver,  fg="blue",  text="Volver",
                font=("Yu Gothic",10,"bold") ,anchor="center",
                width=10,height=1).place(x = 130,y = 420)
    Vprediccion.mainloop()

def grafica():
    contenido = os.listdir('Data/Documentos')
    Vgrafica=Tk()
    Vgrafica.resizable(0,0)
    Vgrafica.geometry("280x300+%d+50" % ((Vgrafica.winfo_screenwidth()-350)/2))
    Vgrafica.title("Menu Principal")    
    def abrir():
        global documento
        documento = str(contenido[len(contenido)-1])
        doc()
    def select():
        global documento
        documento = seleccion.get()
        doc()
    def doc():
        Vgrafica.destroy()
        archivo = open('Data/Documentos/'+str(documento),'r')
        Matriz = []
        total_objetos = 0
        while True:
            hoja = archivo.readline()
            if not hoja:
                break
            lista = list(hoja.split(','))
            Matriz.append(lista)
            total_objetos = total_objetos + 1
        total_objetos = int(total_objetos/5)
        archivo.close()
        for i in range(total_objetos):
            globals()["medidasx"+str(i)] = []
            globals()["medidasy"+str(i)] = []
            globals()["prediccionx"+str(i)] = []
            globals()["predicciony"+str(i)] = []
            globals()["tamaño"+str(i)] = []

############################################            
            globals()["medidasz"+str(i)] = []
            globals()["prediccionz"+str(i)] = []
##################################################
            
            globals()["medidasxstr"+str(i)] = Matriz[7*i]
            globals()["medidasystr"+str(i)] = Matriz[(7*i)+1]
            globals()["prediccionxstr"+str(i)] = Matriz[(7*i)+2]
            globals()["prediccionystr"+str(i)] = Matriz[(7*i)+3]
            globals()["tamañostr"+str(i)] = Matriz[(7*i)+4]

##########################################            
            globals()["medidaszstr"+str(i)] = Matriz[(7*i)+5]
            globals()["prediccionzstr"+str(i)] = Matriz[(7*i)+6]
####################################################
            
            globals()["medidasxm"+str(i)] = 0
            globals()["medidasym"+str(i)] = 0
            globals()["prediccionxm"+str(i)] = 0
            globals()["prediccionym"+str(i)] = 0

######################            
            globals()["medidaszm"+str(i)] = 0
            globals()["prediccionzm"+str(i)] = 0
##############################################3            
##            print(globals()["medidaszstr"+str(i)])
##            print(globals()["prediccionzstr"+str(i)])

            
            for j in range(len(globals()["medidasxstr"+str(i)])-1):
                if int(globals()["medidasxstr"+str(i)][j])>globals()["medidasxm"+str(i)]:
                    globals()["medidasxm"+str(i)] = int(globals()["medidasxstr"+str(i)][j])
                if int(globals()["medidasystr"+str(i)][j])>globals()["medidasym"+str(i)]:
                    globals()["medidasym"+str(i)] = int(globals()["medidasystr"+str(i)][j])
################################################################
                if int(globals()["medidaszstr"+str(i)][j])>globals()["medidaszm"+str(i)]:
                    globals()["medidaszm"+str(i)] = int(globals()["medidaszstr"+str(i)][j])
##################################################################
                
                
            for j in range(len(globals()["medidasxstr"+str(i)])-1):
                globals()["medidasx"+str(i)].append(int(globals()["medidasxstr"+str(i)][j]))
                globals()["medidasy"+str(i)].append(int(globals()["medidasystr"+str(i)][j]))
                globals()["prediccionx"+str(i)].append(int(globals()["prediccionxstr"+str(i)][j]))
                globals()["predicciony"+str(i)].append(int(globals()["prediccionystr"+str(i)][j]))
                globals()["tamaño"+str(i)].append(int(globals()["tamañostr"+str(i)][j].split("/")[0]))
                globals()["tiempo"+str(i)]= float(globals()["tamañostr"+str(i)][j].split("/")[1])
                
                
####################################################################                
                globals()["medidasz"+str(i)].append(int(globals()["medidaszstr"+str(i)][j]))
                globals()["prediccionz"+str(i)].append(int(globals()["prediccionzstr"+str(i)][j]))
##                print(int(globals()["medidaszstr"+str(i)][j]),int(globals()["prediccionzstr"+str(i)][j]),globals()["medidaszm"+str(i)])
##################################################################                
##            print( globals()["medidasz"+str(i)],globals()["prediccionz"+str(i)])
            plt.figure("Objeto "+str(globals()["medidasxstr"+str(i)][len(globals()["medidasxstr"+str(i)])-1]))
            
            plt.subplot(2,2,1)
            plt.title("Tiempo total de la prueba: "+str(globals()["tiempo"+str(i)])+str ("seg"), position=(1, 0),
                    fontdict={'family': 'serif', 
                    'color' : 'darkblue',
                    'weight': 'bold',
                    'size': 18})
            plt.ylabel("Eje X")
            plt.xlabel("Frames/tiempo")
            plt.plot(globals()["tamaño"+str(i)],globals()["medidasx"+str(i)],label="medidax")
            plt.plot(globals()["tamaño"+str(i)],globals()["prediccionx"+str(i)],label="prediccionx")
            plt.legend()
            plt.subplot(2,2,2)
            plt.ylabel("Eje y")
            plt.xlabel("Frames/tiempo")
            plt.plot(globals()["tamaño"+str(i)],globals()["medidasy"+str(i)],label="mediday")
            plt.plot(globals()["tamaño"+str(i)],globals()["predicciony"+str(i)],label="predicciony")
            plt.legend()
############################################################            
            plt.subplot(2,2,3)
            plt.ylabel("Eje z")
            plt.xlabel("Frames/tiempo")
            plt.plot(globals()["tamaño"+str(i)],globals()["medidasz"+str(i)],label="medidaz")
            plt.plot(globals()["tamaño"+str(i)],globals()["prediccionz"+str(i)],label="prediccionz")
            plt.legend()
##############################################################            
            plt.subplot(2,2,4)
            pre = 0
            med = 0
            for k in globals()["prediccionx"+str(i)]:
                if int (k)>pre : pre = int(k)
            for j in globals()["medidasx"+str(i)]:
                if int (j)>med : med = int(j)
            for k in range (len(globals()["prediccionx"+str(i)])):
##                globals()["prediccionx"+str(i)][k] = (-1*int(globals()["prediccionx"+str(i)][k]))+pre
                globals()["predicciony"+str(i)][k] = (-1*int(globals()["predicciony"+str(i)][k]))+med
            for j in range (len(globals()["medidasx"+str(i)])):
##                globals()["medidasx"+str(i)][j] = (-1*int(globals()["medidasx"+str(i)][j]))+pre
                globals()["medidasy"+str(i)][j] = (-1*int(globals()["medidasy"+str(i)][j]))+med
            
            print(pre,med)
            plt.ylabel("Eje y")
            plt.xlabel("Eje x")
            plt.plot(globals()["medidasx"+str(i)],globals()["medidasy"+str(i)],label="medidaxy")
            plt.plot(globals()["prediccionx"+str(i)],globals()["predicciony"+str(i)],label="prediccionxy")
            plt.legend()
            plt.show()

            #Creacion del objeto figura
            fig = plt.figure()
            #Configuracion para proyeccion en 3d
            ax1 = fig.add_subplot(111,projection='3d')
            #Titulo de la grafica
            plt.title("Medidas (verde) vs Prediccion (rojo)",
                    fontdict={'family': 'serif', 
                    'color' : 'darkblue',
                    'weight': 'bold',
                    'size': 18})
            #Dibujo de datos de medicion
            ax1.plot_wireframe(np.array([globals()["medidasx"+str(i)]]),
                               np.array([globals()["medidasy"+str(i)]]),
                               np.array([globals()["medidasz"+str(i)]]),
                               color="g",label="medidas")
            #Dibujo de datos de prediccion
            ax1.plot_wireframe(np.array([globals()["prediccionx"+str(i)]]),
                               np.array([globals()["predicciony"+str(i)]]),
                               np.array([globals()["prediccionz"+str(i)]]),
                               color="r",label="prediccion")
            #Titulo para eje X y Y
            plt.ylabel("Eje y")
            plt.xlabel("Eje x")
            # Muestra del gráfico
            plt.show()

            

    def volver():
        Vgrafica.destroy()
    seleccion = StringVar()
    lista_desplegable = ttk.Combobox(Vgrafica,width=30,values=contenido,
                                     textvariable = seleccion).place(x = 30, y = 180)

    btn1=Button(Vgrafica, command=abrir,  fg="blue",  text="Abrir",font=("Yu Gothic",10,"bold") ,
                anchor="center",width=10,height=1).place(x = 130,y = 70)
    btn2=Button(Vgrafica, command=volver,  fg="blue",  text="Volver",font=("Yu Gothic",10,"bold") ,
                anchor="center",width=10,height=1).place(x = 30,y = 70)
    
    Lab1=Label(Vgrafica,text="Ultima Prueba realizada # "+str(len(contenido)), fg="blue",
               font=("Yu Gothic",12,"bold"),anchor="center").place(x=25,y=20)
    Lab1=Label(Vgrafica,text="o", fg="blue",  font=("Yu Gothic",14,"bold"),anchor="center").place(x=120,y=110)
    Lab1=Label(Vgrafica,text="Seleccione otra Prueba ", fg="blue",  font=("Yu Gothic",12,"bold")
               ,anchor="center").place(x=25,y=140)
   
    btn1=Button(Vgrafica, command=select,  fg="blue",  text="Seleccionar",font=("Yu Gothic",10,"bold") ,
                anchor="center",width=10,height=1).place(x = 130,y = 220)
    btn2=Button(Vgrafica, command=volver,  fg="blue",  text="Volver",font=("Yu Gothic",10,"bold") ,
                anchor="center",width=10,height=1).place(x = 30,y = 220)
    Vgrafica.mainloop()
  
def menu_inicial():
    Vmenu=Tk()
    Vmenu.resizable(0,0)
    Vmenu.geometry("+%d+50" % ((Vmenu.winfo_screenwidth()-350)/2))
    Vmenu.title("Menu Principal")
    def ayuda():
        os.startfile('Data\Prueba.xls')
    def A1():
        Vmenu.destroy()
        umbral()
    def A2():
        Vmenu.destroy()
        prediccion()
    def A3():
        Vmenu.destroy()
        grafica()
    def salir():
        global a
        a = 1
        Vmenu.destroy()
    
    menu=Menu(Vmenu)
    menu_desplegable=Menu(menu,tearoff=0);
    menu.add_cascade(label="Archivo",menu=menu_desplegable)
    menu_desplegable.add_command(label="Ayuda",command=ayuda);
    menu_desplegable.add_separator();
    menu_desplegable.add_command(label="Salir",command=salir)
    Vmenu.config(menu=menu)
    
    btn1=Button(Vmenu, command=A1,  fg="blue",  text="UMBRALIZACION",
                font=("Yu Gothic",20,"bold") ,  anchor="center",
                width=20,height=2).grid(row=0,column=0)
    btn2=Button(Vmenu, command=A2,  fg="blue",  text="PREDICCION \nSEGUIMIENTO",
                font=("Yu Gothic",20,"bold"),anchor="center",
                width=20,height=2).grid(row=1,column=0)
    btn3=Button(Vmenu, command=A3,  fg="blue",  text="GRAFICAS \nRESULTADOS",
                font=("Yu Gothic",20,"bold"),anchor="center",
                width=20,height=2).grid(row=2,column=0)
    btn4=Button(Vmenu, command=salir, fg="blue",text="SALIR",
                font=("Yu Gothic",20,"bold"),anchor="center",
                width=20,height=2).grid(row=4,column=0)
    Vmenu.mainloop()

a = 0
while (a==0):
    menu_inicial()




    








