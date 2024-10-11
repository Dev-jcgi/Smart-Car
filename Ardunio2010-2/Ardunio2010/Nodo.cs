using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Ardunio2010
{
    class Nodo
    {
        public int h;
        public string etiqueta;
        public bool visitado;
        public int distancia;
        public int verticePadre;
        public int id;
        public int color;

        public Nodo(string s, int n, int id, int dis)
        {
            this.id = id;
            etiqueta = s;
            h = n;
            visitado = false;
            distancia = dis;
            verticePadre = -1;
        }
    }
}
