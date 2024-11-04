import numpy as np
from scipy.sparse import *
from scipy.sparse.linalg import eigs, spsolve
import openmesh
import argparse
import os
import polyscope as ps



#Esta función calcula el ángulo entre dos aristas u y v
def myangle(u,v):
    du = np.linalg.norm(u)
    dv = np.linalg.norm(v)

    du = max(du, 1e-8)
    dv = max(dv, 1e-8)

    return np.arccos(np.dot(u,v)/(du*dv))

#Esta función computa la matriz Laplaciana de una malla
#Como la matriz Laplaciana tiene muchos elementos ceros, se utiliza una matriz dispersa (lil_matrix)
def laplacian(mesh):
    n = mesh.n_vertices() #Num. vértices de la malla
    print(f"Num. vertices {n}")
    W = lil_matrix((n,n), dtype=float) #Se crea una matriz dispersa de n x n
    print(W.shape)

    points = mesh.points() #Se obtienen las coordenadas de los vértices de la malla

    #Para cada vertice de la malla
    for i,v in enumerate(mesh.vertices()):
        f_it = openmesh.VertexFaceIter(mesh, v) #Se obtienen las caras que comparten el vértice v
        for f in f_it: #Para cada cara f
            v_it = openmesh.FaceVertexIter(mesh,f) #Se obtienen los vértices de la cara f
            L = [] 
            for vv in v_it: # Se obtienen los vértices compartidos para esa cara
                if vv.idx()!=i:
                    L.append(vv.idx())
            j = L[0]
            k = L[1]

            #Se obtienen las coordenadas de los vértices en una cara
            vi = points[i,:]
            vj = points[j,:]
            vk = points[k,:]

            #Se calculan los ángulos alpha y beta
            alpha = myangle(vi-vk, vj-vk)
            beta = myangle(vi-vj,vk-vj)

            #Se acumulan los cotangentes para las aristas ij e ik
            W[i,j] = W[i,j] + 1.0/(2.0*np.tan(alpha))
            W[i,k] = W[i,k] + 1.0/(2.0*np.tan(beta))
    
    #Se suman todas las filas de la matriz W y se calcula la inversa de cada suma
    S = 1.0/W.sum(axis=1)
    print(S.shape)

    #Se calcula la matriz Laplaciana normalizada. La diagonal es 1 y los demás elementos son -1/n. La idea es que
    #la suma de cada fila sea 0
    W = eye(n,n)-spdiags(np.squeeze(S),0,n,n)*W
    return W

def uniform_laplacian(mesh):
    n = mesh.n_vertices() #Num. vértices de la malla
    print(f"Num. vertices {n}")
    W = lil_matrix((n,n), dtype=float) #Se crea una matriz dispersa de n x n
    print(W.shape)

    #Para cada vertice de la malla
    for i,v in enumerate(mesh.vertices()):
        f_it = openmesh.VertexFaceIter(mesh, v) #Se obtienen las caras que comparten el vértice v
        for f in f_it: #Para cada cara f
            v_it = openmesh.FaceVertexIter(mesh,f) #Se obtienen los vértices de la cara f
            L = [] 
            for vv in v_it: # Se obtienen los vértices compartidos para esa cara
                if vv.idx()!=i:
                    L.append(vv.idx())
            j = L[0]
            k = L[1]

            W[i,j] = 1
            W[i,k] = 1
            
    
    #Se suman todas las filas de la matriz W y se calcula la inversa de cada suma
    S = 1.0/W.sum(axis=1)
    print(S.shape)

    #Se calcula la matriz Laplaciana normalizada. La diagonal es 1 y los demás elementos son -1/n. La idea es que
    #la suma de cada fila sea 0
    W = eye(n,n)-spdiags(np.squeeze(S),0,n,n)*W
    return W

def mean_value_laplacian(mesh):
    n = mesh.n_vertices() #Num. vértices de la malla
    print(f"Num. vertices {n}")
    W = lil_matrix((n,n), dtype=float) #Se crea una matriz dispersa de n x n
    print(W.shape)

    points = mesh.points() #Se obtienen las coordenadas de los vértices de la malla

    #Para cada vertice de la malla
    for i,v in enumerate(mesh.vertices()):
        f_it = openmesh.VertexFaceIter(mesh, v) #Se obtienen las caras que comparten el vértice v
        for f in f_it: #Para cada cara f
            v_it = openmesh.FaceVertexIter(mesh,f) #Se obtienen los vértices de la cara f
            L = [] 
            for vv in v_it: # Se obtienen los vértices compartidos para esa cara
                if vv.idx()!=i:
                    L.append(vv.idx())
            j = L[0]
            k = L[1]

            #Se obtienen las coordenadas de los vértices en una cara
            vi = points[i,:]
            vj = points[j,:]
            vk = points[k,:]

            #Se calculan los ángulos alpha y beta
            gamma = myangle(vj-vi, vk-vi)
            dist_vi_vj = np.linalg.norm(vi - vj)
            dist_vi_vk = np.linalg.norm(vi - vk)

            #Se acumulan los cotangentes para las aristas ij e ik
            W[i,j] = W[i,j] + np.tan(gamma/2.0)/(2.0*dist_vi_vj)
            W[i,k] = W[i,k] + np.tan(gamma/2.0)/(2.0*dist_vi_vk)
    
    #Se suman todas las filas de la matriz W y se calcula la inversa de cada suma
    S = 1.0/W.sum(axis=1)
    print(S.shape)

    #Se calcula la matriz Laplaciana normalizada. La diagonal es 1 y los demás elementos son -1/n. La idea es que
    #la suma de cada fila sea 0
    W = eye(n,n)-spdiags(np.squeeze(S),0,n,n)*W
    return W

#Esta función calcula el borde de una malla. Recibe como parámetros la matriz F de caras y el número de vértices
def compute_boundary(F, numVert):
    numFaces = F.shape[0]

    #Creamos una matriz de adyacencia para vértices
    A = np.zeros((numVert, numVert))

    #Se suma un valor a A[i,j] si los vértices i y j son adyacentes
    for i in range(numFaces):
        f = F[i,:]
        A[f[0], f[1]] = A[f[0], f[1]] + 1
        A[f[0], f[2]] = A[f[0], f[2]] + 1
        A[f[2], f[1]] = A[f[2], f[1]] + 1
    
    #Como la malla es un grafo no dirigido, también consideramos lo simétrico
    A = A + A.T

    #Una arista de frontera es aquella que tiene un valor 1 en la matriz A
    result = np.where(A==1)
    boundary = [result[1][0], result[0][0]] #Calculamos una primera arista de frontera

    #A partir de la primera arista de frontera, se calcula el resto del borde
    s = boundary[1]
    i = 1

    while i < numVert:
        #Buscar algún elemento con valor 1 en la fila s
        vals = []
        for j in range(numVert):
            if A[s,j] == 1:
                vals.append(j)
        
        assert(len(vals)==2)
        if vals[0] == boundary[i-1]:
            s = vals[1]
        else:
            s = vals[0]
        
        if s!=boundary[0]:
            boundary.append(s)
        else:
            break
        i = i + 1
    return boundary


def param_coords(L, boundary, xy_boundary):
        #Removemos las filas de los vértices de la frontera
    for i in boundary:
        L[i,:] = 0
        L[i,i] = 1

    #Creamos un vector X en donde de calcularán las coordenadas X de la parametrización
    x = np.zeros((numVert,1))
    #Se asignan las coordenadas X de los vértices de la frontera
    for i,b in enumerate(boundary):
        x[b,0] = xy_boundary[i,0]

    #Se resuelve el sistema de ecuaciones
    xx = spsolve(L, x)

    #Hacemos lo mismo para coordenadas Y
    y = np.zeros((numVert,1))
    for i,b in enumerate(boundary):
        y[b,0] = xy_boundary[i,1]
    yy = spsolve(L, y)

    #Se crea un arreglo de numpy con las coordenadas X e Y de la parametrización, se agregan coordenadas Z = 1 para poder dibujar
    coord = np.column_stack((xx,yy, np.ones(numVert)))

    #Creamos un array con solamente las coordenadas X, Y de la parametrización. La usamos para dibujar en Polyscope
    coord2 = np.column_stack((xx,yy))
    return coord, coord2

#El script recibe como parámetro el nombre de un archivo .off
parser = argparse.ArgumentParser()
parser.add_argument("--file", type=str, default="", help="")
opt = parser.parse_args()

#Usamos OpenMesh para leer la malla
mesh = openmesh.read_trimesh(opt.file)
points = mesh.points()
numVert = points.shape[0]

#Calculamos la frontera
boundary = compute_boundary(mesh.face_vertex_indices(), points.shape[0])
numBound = len(boundary)

#Calculamos la longitud de la frontera
d = 0
lastBound = boundary[-1]

for i in boundary:
    d = d + np.linalg.norm(points[i,:] - points[lastBound,:])
    lastBound = i

#Se calcula la parametrización de la frontera en una circunferencia, usando como guía la longitud de cada arista de la frontera
print("Chord length:", d)
vb = points[boundary,:]
sel = list(range(1,numBound))
sel.append(0)
vb2 = vb[sel,:]

D = np.cumsum(np.linalg.norm(vb2-vb, axis=1))
t = (D - D[0])/d
xy_boundary = np.hstack([np.expand_dims(np.cos(2*np.pi*t),axis=1), np.expand_dims(np.sin(2*np.pi*t),axis=1)])
print(xy_boundary)

#Se calcula la matriz Laplaciana
L1 = laplacian(mesh)
L2 = mean_value_laplacian(mesh)
L3 = uniform_laplacian(mesh)

lap_coords, lap_coords2 = param_coords(L1, boundary, xy_boundary)
print(lap_coords.shape)

mv_lap_coords, mv_lap_coords2 = param_coords(L2, boundary, xy_boundary)
print(mv_lap_coords.shape)

unif_lap_coords, unif_lap_coords2 = param_coords(L3, boundary, xy_boundary)
print(unif_lap_coords.shape)

#Dibujamos en Polyscope
ps.init()
#Malla original con la parametrización. Internamente Polyscope aplica una textura usando la parametrización
ps_mesh = ps.register_surface_mesh("mesh", points, mesh.face_vertex_indices())
ps_mesh.add_parameterization_quantity("parameterization with harmonic laplacian", lap_coords2, defined_on='vertices', enabled=True)
ps_mesh.add_parameterization_quantity("parameterization with mean value laplacian", mv_lap_coords2, defined_on='vertices', enabled=True)
ps_mesh.add_parameterization_quantity("parameterization with uniform laplacian", unif_lap_coords2, defined_on='vertices', enabled=True)

#Mostramos la parametrización como si fuera una superficie más
ps_mesh2 = ps.register_surface_mesh("mesh2 harmonic laplacian", lap_coords, mesh.face_vertex_indices())
ps_mesh3 = ps.register_surface_mesh("mesh2 mean value laplacian", mv_lap_coords, mesh.face_vertex_indices())
ps_mesh3 = ps.register_surface_mesh("mesh2 uniform laplacian", unif_lap_coords, mesh.face_vertex_indices())
ps.show()