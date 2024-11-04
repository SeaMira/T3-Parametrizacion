# T3-Parametrizacion

Tarea 3 del ramo de Procesamiento Geométrico y Análisis de Datos. Se parametrizarán mallas triangulares 3D para poder mapear una textura de tablero de ajedrez en 3 modelos utilizando diferentes formas de discretización del Laplaciano.

Para poder ejecutar el programa se necesita abrir una terminal en el directorio principal del proyecto y ejecutar el siguiente comando:

```bash
python parameterization.py --file "filepath"
```

--file: Dirección del archivo con la nube de puntos, extensión .off

## Resultados

### Bunny

Para poder obtener una parametrización del conejo en bunny.off se ejecuta el siguiente comando:

```bash
python parameterization.py --file "models/bunny.off"
```

- Parametrización de malla en circunferencia para el laplaciano armónico:

![Parametrización de malla en circunferencia para el laplaciano armónico.](./images/bunny_2d_har.png)

![Lado izquierdo de texturización con laplaciano armónico.](./images/bunny_3d_left_har.png)
![Frente de texturización con laplaciano armónico.](./images/bunny_3d_front_har.png)
![Lado derecho de texturización con laplaciano armónico.](./images/bunny_3d_right_har.png)

- Parametrización de malla en circunferencia para el laplaciano de valor medio:

![Parametrización de malla en circunferencia para el laplaciano de valor medio.](./images/bunny_2d_mv.png)

![Lado izquierdo de texturización con laplaciano de valor medio.](./images/bunny_3d_left_mv.png)
![Frente de texturización con laplaciano de valor medio.](./images/bunny_3d_front_mv.png)
![Lado derecho de texturización con laplaciano de valor medio.](./images/bunny_3d_right_mv.png)

- Parametrización de malla en circunferencia para el laplaciano de peso uniforme:

![Parametrización de malla en circunferencia para el laplaciano de peso uniforme.](./images/bunny_2d_uni.png)

![Lado izquierdo de texturización con laplaciano uniforme.](./images/bunny_3d_left_uni.png)
![Frente de texturización con laplaciano uniforme.](./images/bunny_3d_front_uni.png)
![Lado derecho de texturización con laplaciano uniforme.](./images/bunny_3d_right_uni.png)

Con el laplaciano de pesos uniforme se notan deformaciones, tanto en los triángulos como en los cuadrados de la textura, que son expandidos y tienen curvas sin tomar en cuenta la distribución de triángulos de la malla original, sino la de la parametrización. Esto ocurre en todos los modelos a continuación.

Entre el laplaciano con pesos armónicos y pesos de valor medio se notan buenos resultados, se reconocen cuadrilateros de tamaños consistentes, sin embargo, en zonas de triángulos delgados se nota la diferencia y el laplaciano con discretización de cotangentes muestra un mejor resultado al no "estirar" los cuadrados y hacerlos más pequeños.

### Nefertiti

Para poder obtener una parametrización del conejo en nefertiti.off se ejecuta el siguiente comando:

```bash
python parameterization.py --file "models/nefertiti.off"
```

- Parametrización de malla en circunferencia para el laplaciano armónico:

![Parametrización de malla en circunferencia para el laplaciano armónico.](./images/nef_2d_har.png)

![Lado izquierdo de texturización con laplaciano armónico.](./images/nef_3d_har.png)

- Parametrización de malla en circunferencia para el laplaciano de valor medio:

![Parametrización de malla en circunferencia para el laplaciano de valor medio.](./images/nef_2d_mv.png)

![Lado izquierdo de texturización con laplaciano de valor medio.](./images/nef_3d_mv.png)

- Parametrización de malla en circunferencia para el laplaciano de peso uniforme:

![Parametrización de malla en circunferencia para el laplaciano de peso uniforme.](./images/nef_2d_uni.png)

![Lado izquierdo de texturización con laplaciano uniforme.](./images/nef_3d_uni.png)

Deformaciones similares a las del conejo, pero solo cuadrados más anchos que largos y viceversa.

Entre el laplaciano con pesos armónicos y pesos de valor medio se notan buenos resultados nuevamente, se reconocen cuadrilateros de tamaños consistentes, sin embargo, esta vez no hay mucho triángulos delgados y aún así en el método con valor medio se notan cuadrados más expandidos en zonas de mayor curvatura (nariz, por ejemplo), por lo que el laplaciano con cotangentes es levemente de mejor calidad.

### Lucy

Para poder obtener una parametrización del conejo en lucy.off se ejecuta el siguiente comando:

```bash
python parameterization.py --file "models/lucy.off"
```

- Parametrización de malla en circunferencia para el laplaciano armónico:

![Parametrización de malla en circunferencia para el laplaciano armónico.](./images/lucy_2d_har.png)

![Frente de texturización con laplaciano armónico.](./images/lucy_3d_front_har.png)
![Parte trasera de texturización con laplaciano armónico.](./images/lucy_3d_back_har.png)

- Parametrización de malla en circunferencia para el laplaciano de valor medio:

![Parametrización de malla en circunferencia para el laplaciano de valor medio.](./images/lucy_2d_mv.png)

![Frente de texturización con laplaciano de valor medio.](./images/lucy_3d_front_mv.png)
![Parte trasera de texturización con laplaciano de valor medio.](./images/lucy_3d_back_mv.png)

- Parametrización de malla en circunferencia para el laplaciano de peso uniforme:

![Parametrización de malla en circunferencia para el laplaciano de peso uniforme.](./images/lucy_2d_uni.png)

![Frente de texturización con laplaciano uniforme.](./images/lucy_3d_front_uni.png)
![Parte trasera de texturización con laplaciano uniforme.](./images/lucy_3d_back_uni.png)

Mismas deformaciones señaladas en el caso del conejo. Cuadrados con lados curvos, estirados, más anchos que largos, etc.

Entre el laplaciano con pesos armónicos y pesos de valor medio se notan buenos resultados nuevamente, se reconocen cuadrilateros de tamaños consistentes. Al igual que en el caso del conejo y nefertiti, en zonas de mayor curvatura y de más triángulos el laplaciano con valor medio presenta cuadriláteros de textura más similares a un rombo que un cuadrado. Son de buena calidad ambos pero sigue siéndolo más el armónico.

Se cree que la diferencia primordial entre los mejores métodos (armónico y de valor medio) está en que el último no toma en cuenta el tamaño total de los triángulos adyacentes a cada una de las aristas que salen de cada vértice revisado.

## Author

Sebastián Mira Pacheco
