using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Ardunio2010
{
    public class Autom
    {
        static bool val = false;
        static private int code = 0;
        public String obtener(String c)
        {
            c = format(c);
            val = longitud(c);
            if(!val){
			        c = c + " Cadena incorrecta: Longitud incorrecta.";
			        return c;
		    }
		    if(code==0){
                    c = c + " Cadena incorrecta: Operación invalida.";
                    return c;
		    }
            //c = c + " código: " + code;
            return code.ToString();
        }
        public String format(String s)
        {
            s = s.ToUpper().Trim();
            String s2 = "";
            if (s.Length > 2)
            {
                s2 = s.Substring(0, 2);
            }
            if (s2.Equals("NO"))
            {
                    String s3 = s.Substring(2).Trim();
                    s = s2 + " " + s3;
                    return s;
            }
            return s;
        }
        public bool longitud(String s)
        {
            bool a = false;
            int lon = s.Length;
            if (lon == 0 || lon > 8)
            {
                a = false;
                return a;
            }
            else
                a = true;
            switch (lon)
            {
                case 1:
                    codeu(s);
                    break;
                case 2:
                    coded(s);
                    break;
                case 3:
                    codet(s);
                    break;
                case 4:
                    codecu(s);
                    break;
                case 5:
                    codeci(s);
                    break;
                case 6:
                    codes(s);
                    break;
                case 7:
                    codesi(s);
                    break;
                case 8:
                    codeoc(s);
                    break;
                default:
                    return a = false;
            }
            return a;
        }
        //Funciones dependiendo de la longitud
        public void codeu(String s)
        {
            int n = 0;
            switch (s)
            {
                case "R":
                    n = 1;
                    break;
                case "U":
                    n = 2;
                    break;
                case "D":
                    n = 3;
                    break;
                case "L":
                    n = 4;
                    break;
                case "S":
                    n = 5;
                    break;
                default:
                    n = 0;
                    break;
            }
            code = n;
        }

        public void coded(String s)
        {
            int n = 0;
            switch (s)
            {
                case "RI":
                    n = 1;
                    break;
                case "UP":
                    n = 2;
                    break;
                case "DO":
                    n = 3;
                    break;
                case "LE":
                    n = 4;
                    break;
                case "ST":
                    n = 5;
                    break;
                default:
                    n = 0;
                    break;
            }
            code = n;
        }

        public void codet(String s)
        {
            int n = 0;
            switch (s)
            {
                case "RIG":
                    n = 1;
                    break;
                case "DOW":
                    n = 3;
                    break;
                case "LEF":
                    n = 4;
                    break;
                case "STO":
                    n = 5;
                    break;
                default:
                    n = 0;
                    break;
            }
            code = n;
        }
        public void codecu(String s)
        {
            int n = 0;
            switch (s)
            {
                case "RIGH":
                    n = 1;
                    break;
                case "DOWN":
                    n = 3;
                    break;
                case "LEFT":
                    n = 4;
                    break;
                case "NO R":
                    n = -1;
                    break;
                case "NO U":
                    n = -2;
                    break;
                case "NO D":
                    n = -3;
                    break;
                case "NO L":
                    n = -4;
                    break;
                case "STOP":
                    n = 5;
                    break;
                default:
                    n = 0;
                    break;
            }
            code = n;
        }
        public void codeci(String s)
        {
            int n = 0;
            switch (s)
            {
                case "RIGHT":
                    n = 1;
                    break;
                case "NO RI":
                    n = -1;
                    break;
                case "NO UP":
                    n = -2;
                    break;
                case "NO DO":
                    n = -3;
                    break;
                case "NO LE":
                    n = -4;
                    break;
                default:
                    n = 0;
                    break;
            }
            code = n;
        }

        public void codes(String s)
        {
            int n = 0;
            switch (s)
            {
                case "NO RIG":
                    n = -1;
                    break;
                case "NO DOW":
                    n = -3;
                    break;
                case "NO LEF":
                    n = -4;
                    break;
                default:
                    n = 0;
                    break;
            }
            code = n;
        }

        public void codesi(String s)
        {
            int n = 0;
            switch (s)
            {
                case "NO RIGH":
                    n = -1;
                    break;
                case "NO DOWN":
                    n = -3;
                    break;
                case "NO LEFT":
                    n = -4;
                    break;
                default:
                    n = 0;
                    break;
            }
            code = n;
        }

        public void codeoc(String s)
        {
            int n = 0;
            if (s.Equals("NO RIGHT"))
            {
                n = -1;
            }
            code = n;
        }
    }
}
