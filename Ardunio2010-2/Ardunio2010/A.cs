using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Ardunio2010
{
    class A
    {
        private int MAX_VERTS = 20;
        private int INFINITY = 1000000;
        private Nodo[] vertexList;
        private int[,] adjMat;
        private int nVerts;
        int[] color;
        int[] d;
        public List<Nodo> abierta = new List<Nodo>();
        public List<Nodo> cerrada = new List<Nodo>();
        int inicio;
        int meta;

        public A()
        {
            vertexList = new Nodo[MAX_VERTS];
            adjMat = new int[MAX_VERTS, MAX_VERTS];
            nVerts = 0;
            for (int j = 0; j < MAX_VERTS; j++)
                for (int k = 0; k < MAX_VERTS; k++)
                    adjMat[j, k] = INFINITY;
        }

        public void setInicio(String ini)
        { inicio = buscarNodo(ini); }

        public int getMeta()
        { return meta; }

        public void setMeta(String met)
        { meta = buscarNodo(met); }

        public int buscarNodo(string s)
        {
            for (int i = 0; i < nVerts; i++)
            {
                if (vertexList[i].etiqueta == s)
                {
                    return i;
                }
            }
            return -1;//era 0 verificar funcionalidad
        }

        //funcion para vaciar grafo
        public void vaciarVertex()
        {
            for (int i = 0; i < nVerts; i++)
            {
                vertexList[nVerts]=null;
            }
            nVerts = 0;
        }

        //funcion para agregar vertice
        public bool addVertex(String s, int h)
        {
            bool existe = false;
            for (int i = 0; i < nVerts; i++)
            {
                if (vertexList[i].etiqueta == s)
                    existe = true;
            }
            if (!existe)//se valida la etiqueta del vertice
            {
                vertexList[nVerts] = new Nodo(s, h, nVerts, INFINITY);
                nVerts++;
            }
            else
            {
                Console.WriteLine("La etiqueta para este vertice ya existe");
            }
            return existe;
        }

        //funcion para agregar relaiones
        public void addEdge(String st, String en, int weight)
        {
            int start = -1, end = -1;
            for (int i = 0; i < nVerts; i++)
            {
                if (vertexList[i].etiqueta == st)
                    start = i;
                if (vertexList[i].etiqueta == en)
                    end = i;
            }
            if (start != -1 && end != -1)
            {
                adjMat[start, end] = weight;
                adjMat[end, start] = weight;
            }
            else
            {
                Console.WriteLine("No existen los nodos seleccionados");
            }
        }

        public String Matriz()
        {
            String mat = "";
            for (int i = 0; i < nVerts + 1; i++)
            {
                for (int j = 0; j < nVerts + 1; j++)
                {
                    if (i == 0)
                    {
                        if (j == 0)
                        {
                            mat += "\t";
                        }
                        else
                        {
                            mat += vertexList[j - 1].etiqueta + "\t";
                        }
                    }
                    else
                    {
                        if (j == 0)
                        {
                            mat += vertexList[i - 1].etiqueta + "\t";
                        }
                        else
                        {
                            if (adjMat[i - 1, j - 1] >= INFINITY)
                                mat += "-\t";
                            else {
                                mat += adjMat[i - 1, j - 1] + "\t";
                            }
                        }
                    }

                }
                mat += Environment.NewLine;
            }
            return mat;
        }

        public string Anchura()
        {
            color = new int[nVerts];
            d = new int[nVerts];

            for (int i = 0; i < nVerts; i++)
            {
                color[i] = 0;
                vertexList[i].color = 0;
                vertexList[i].verticePadre = -1;
                d[i] = 0;
            }
            abierta.Clear();
            vertexList[inicio].color = 1;
            visita(inicio);

            for (int i = 0; i < nVerts; i++)
            {
                if (color[i] == 0)
                {
                    color[i] = 1;
                    visita(i);
                }
            }
            if (vertexList[meta].verticePadre != -1)
                return ruta(vertexList[meta]);
            else
                return "No hay solucion";

        }

        public void visita(int i)
        {
            color[i] = 2;
            vertexList[i].color = 2;
            for (int j = 0; j < nVerts; j++)
            {
                if (adjMat[i, j] < INFINITY && vertexList[j].color == 0)
                {
                    vertexList[j].verticePadre = i;
                    vertexList[j].color = 1;
                    abierta.Add(vertexList[j]);
                }
            }
            while (abierta.Count > 0)
            {
                Nodo actual = abierta.ElementAt(0);
                abierta.RemoveAt(0);
                visita(actual.id);
            }


        }

        //algortimo
        public void dijkstra()
        {
            List<Nodo> lista = new List<Nodo>();
            Nodo actual;
            lista.Add(vertexList[inicio]);
            vertexList[inicio].distancia = 0;

            while (lista.Count > 0)
            {
                int indiceCola = lista.IndexOf(min());//se busca el nodo no visitado con menor distancia
                actual = lista.ElementAt(indiceCola);//u es ese nodo
                lista.RemoveAt(indiceCola);//se elimina de la lista
                if (actual.visitado) continue;//si el nodo ya fue visitado se busca otro
                actual.visitado = true;//se marca u como visitado
                for (int i = 0; i < nVerts; i++)//se revisa los adyacentes del vertice actual
                {
                    if (adjMat[actual.id, i] < INFINITY)//si es menor que infinito es un adyacente
                    {
                        Nodo adyacente = vertexList[i];//v es el nodo adyacente
                        int peso = adjMat[actual.id, adyacente.id];//peso es el peso entre el nodo u y v
                        if (adyacente.visitado == false) //si  el nodo v no ha sido visitado
                        //if (adyacente.id != actual.verticePadre)
                        {
                            relajacion(actual, adyacente, peso, lista);
                        }
                    }
                }
            }
            limpiar();
            //caminoCorto();
        }

        public string Aasterico()
        {
            Nodo actual;
            vertexList[inicio].distancia = 0;
            abierta.Add(vertexList[inicio]);

            while(abierta.Count>0)
                {
                   abierta=abierta.OrderBy(p => p.h+p.distancia).ToList();
                   actual = abierta.ElementAt(0);

                  if (actual == vertexList[meta])
                      {
                          Console.Write(ruta(actual));
                          return ruta(actual);
                       }

                  abierta.RemoveAt(0);
                  cerrada.Add(actual);
                  for (int i = 0; i < nVerts; i++)//se revisa los adyacentes del vertice actual
                  {
                      if (adjMat[actual.id, i] < INFINITY)//si es menor que infinito es un adyacente
                      {
                          Nodo adyacente = vertexList[i];//v es el nodo adyacente
                          if (cerrada.Contains(adyacente)) 
                          {
                              continue;
                          }

                          int tentativag = actual.distancia + adjMat[actual.id, i];
                          if (!abierta.Contains(adyacente) || tentativag <= adyacente.distancia)
                          {
                              adyacente.verticePadre = actual.id;
                              if(!abierta.Contains(adyacente))
                                  abierta.Add(adyacente);
                            }
                          
                          
                      }
                  }
                }
            return "No hay solucion";
             }


        public string ruta(Nodo actual)
        {
            String variable="";

            if (actual != null)
            {
                if (actual.verticePadre != -1)
                    variable =ruta(vertexList[actual.verticePadre]) + "-" + actual.etiqueta;
                else
                    variable = actual.etiqueta;
                //Console.Write()
            }

            return variable;
        }

        public void relajacion(Nodo actual, Nodo ady, int peso, List<Nodo> lista)
        {
            if (actual.distancia + peso < ady.distancia)//si la distancia del nodo u + el peso entre u y v es menor que la distancia acutal de u 
            {
                ady.distancia = actual.distancia + peso;//la distancia de v = distancia de u + peso entre u y v
                ady.verticePadre = actual.id;//el nodo padre de v es u
                if (!lista.Contains(ady))//si la lista no contiene a v
                    lista.Add(ady);//se agrega v a la lista
            }
        }

        Nodo min()//busca el nodo no visitado con menor distancia
        {
            int minDist = INFINITY;
            int indiceMin = 0;
            for (int j = 0; j < nVerts; j++)
            {
                if (!vertexList[j].visitado && vertexList[j].distancia < minDist)
                {
                    minDist = vertexList[j].distancia;
                    indiceMin = j;
                }
            }
            return vertexList[indiceMin];
        }

        void limpiar()
        {
            for (int i = 0; i < nVerts; i++)//regresa en no visitados a los nodos
            {
                vertexList[i].visitado = false;
                //vertexList[i].verticePadre = -1;
            }
            abierta.Clear();
            cerrada.Clear();
        }

        public void limpiar2()
        {
            for (int i = 0; i < nVerts; i++)//regresa en no visitados a los nodos
            {
                vertexList[i].visitado = false;
                vertexList[i].verticePadre = -1;
                vertexList[i].distancia = INFINITY;
            }
            abierta.Clear();
            cerrada.Clear();
        }

        public String caminoCorto()//recorre los nodos y muestra su nodo padre y el costo minimo para llegar a ellos
        {
            String caminos = "";
            for (int j = 0; j < nVerts; j++)
            {
                caminos += vertexList[j].etiqueta + "=";
                if (vertexList[j].distancia >= INFINITY)
                    caminos += "inf";
                else
                    caminos += vertexList[j].distancia;
                String padre = "";
                if (vertexList[j].verticePadre != -1) //valida que tenga padre
                    padre = vertexList[vertexList[j].verticePadre].etiqueta;
                caminos += "(" + padre + ")\r\n";
            }
            return caminos;
        }

    }


}
