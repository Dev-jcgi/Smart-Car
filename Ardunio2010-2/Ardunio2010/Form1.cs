using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;
using Ardunio2010.Properties;


namespace Ardunio2010
{
    public partial class Form1 : Form
    {
        int valor = 0;//valor enviado al arduino
        A grafo = new A();
        bool anchuraEnc = false;
        String[] anchuraStrNodo;
        int anchuraCont = 0;
        int asteriscoCont = 0;
        bool asteriscoEnc = false;
        String[] asteriscoStrNodo;
        int asteriscoEncCont = 0;

        public Form1()
        {
            InitializeComponent();
            for (int x = 0; x <= 50; x++)
                cbox.Items.Insert(x, x.ToString());
            serialPort1.BaudRate = 9600;
        }

        private void chkAde_CheckedChanged(object sender, EventArgs e)
        {
            if (chkAde.Checked && chkAtr.Checked)
            {
                chkAtr.Checked = false;
            }
            if (chkAde.Checked)
                valor += 1;
            else
                valor -= 1;
            ImpEnv();
        }

        private void chkAtr_CheckedChanged(object sender, EventArgs e)
        {
            if (chkAtr.Checked && chkAde.Checked)
            {
                chkAde.Checked = false;
            }
            if (chkAtr.Checked)
                valor += 2;
            else
                valor -= 2;
            ImpEnv();
        }

        private void chkDer_CheckedChanged(object sender, EventArgs e)
        {
            if (chkDer.Checked && chkIzq.Checked)
            {
                chkIzq.Checked = false;
            }
            if (chkDer.Checked)
                valor += 4;
            else
                valor -= 4;
            ImpEnv();
        }

        private void chkIzq_CheckedChanged(object sender, EventArgs e)
        {
            if (chkDer.Checked && chkIzq.Checked)
            {
                chkDer.Checked = false;
            }
            if (chkIzq.Checked)
                valor += 8;
            else
                valor -= 8;
            ImpEnv();
        }

       

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            //Se muestra un cuadro de dialogo preguntando si se desea cerrar el programa.
            if ((MessageBox.Show("¿Seguro que quiere salir?", "Confirmación", MessageBoxButtons.YesNo, MessageBoxIcon.Question)) == DialogResult.Yes)
            {
                //Si la respuesta es sí se checa si esta abierto el puerto y se cierra, al ser el método Closing se cerrará solo.
                if (serialPort1.IsOpen)
                    serialPort1.Close();
            }
            else
            {
                //Si la respuesta es no, se cancela la salida del programa.
                e.Cancel = true;
            }
        }
        public void ImpEnv()
        {
            textBox1.Text = valor.ToString("X");
            if (serialPort1.IsOpen)
            {
                serialPort1.Write(valor.ToString("X"));
            }
        }

        private void cbox_SelectedIndexChanged(object sender, EventArgs e)
        {
            serialPort1.PortName = "COM" + cbox.SelectedItem;
            chkEnviar.Enabled = true;
        }

        private void chkEnviar_CheckedChanged(object sender, EventArgs e)
        {
            if (chkEnviar.Checked)
            {
                if (!serialPort1.IsOpen)
                {
                    try
                    {
                        serialPort1.Open();
                        cbox.Enabled = false;
                    }
                    catch (Exception ex)
                    {
                        MessageBox.Show("Puerto no válido");
                        chkEnviar.Checked = false;
                    }
                }
            }
            else
            {
                if (serialPort1.IsOpen)
                    serialPort1.Close();
                cbox.Enabled = true;
            }
        }

        private void label3_Click(object sender, EventArgs e)
        {

        }
                       
        private void inputText_KeyPress(object sender, KeyPressEventArgs e)
        {
            if ((e.KeyChar == (char)Keys.Enter))
            {
                String s, cadena;
                s = inputText.Text;
                Autom a = new Autom();
                cadena = a.obtener(s);
                switch (cadena)
                {
                    case "1":
                        chkDer.Checked = true;
                        break;
                    case "2":
                        chkAde.Checked = true;
                        break;
                    case "3":
                        chkAtr.Checked = true;
                        break;
                    case "4":
                        chkIzq.Checked = true;
                        break;
                    case "5":
                        chkDer.Checked = false;
                        chkAde.Checked = false;
                        chkAtr.Checked = false;
                        chkIzq.Checked = false;
                        break;
                    case "-1":
                        chkDer.Checked = false;
                        break;
                    case "-2":
                        chkAde.Checked = false;
                        break;
                    case "-3":
                        chkAtr.Checked = false;
                        break;
                    case "-4":
                        chkIzq.Checked = false;
                        break;
                    default:
                        MessageBox.Show(cadena);
                        break;
                }
                inputText.Clear();
            }
        }

        private void txtIniBus_KeyUp(object sender, KeyEventArgs e)
        {
            if (e.KeyData == Keys.Enter)
            {
                if (grafo.buscarNodo(txtIniBus.Text) != -1)
                {
                    grafo.setInicio(txtIniBus.Text);
                    grafo.dijkstra();
                    txtCamMin.Text = grafo.caminoCorto();
                    txtIniBus.ReadOnly = true;
                    btnReiniBus.Enabled = true;
                    txtMeta.ReadOnly = false;
                }
                else
                    MessageBox.Show("No se encontro el nodo", "Advertencia");
            }
        }

        private void btnGrafoDef_Click(object sender, EventArgs e)
        {
            grafo.vaciarVertex();
            txtRelaciones.Text = "";
            txtListNodos.Text = "";
            ReiniciarBusqueda();
            
            grafo.addVertex("a", 0);//0
            grafo.addVertex("b", 0);//1
            grafo.addVertex("c", 0);//2
            grafo.addVertex("d", 0);//3
            grafo.addVertex("e", 0);//4
            grafo.addVertex("f", 0);//5
            grafo.addVertex("g", 0);//6
            grafo.addVertex("h", 1);//7
            grafo.addVertex("i", 0);//8
            grafo.addVertex("j", 0);//9
            grafo.addVertex("k", 0);//10

            grafo.addEdge("a", "b", 5);
            grafo.addEdge("a", "c", 1);
            grafo.addEdge("a", "d", 3);
            grafo.addEdge("b", "e", 3);
            grafo.addEdge("b", "f", 10);
            grafo.addEdge("c", "f", 10);
            grafo.addEdge("d", "f", 8);
            grafo.addEdge("d", "g", 10);
            grafo.addEdge("e", "f", 2);
            grafo.addEdge("f", "i", 2);
            grafo.addEdge("f", "j", 7);
            grafo.addEdge("g", "j", 2);
            grafo.addEdge("h", "k", 1);
            grafo.addEdge("i", "h", 1);
            grafo.addEdge("j", "k", 5);

            txtListNodos.Text += "a (0)" + "\r\n";
            txtListNodos.Text += "b (0)" + "\r\n";
            txtListNodos.Text += "c (0)" + "\r\n";
            txtListNodos.Text += "d (0)" + "\r\n";
            txtListNodos.Text += "e (0)" + "\r\n";
            txtListNodos.Text += "f (0)" + "\r\n";
            txtListNodos.Text += "g (0)" + "\r\n";
            txtListNodos.Text += "h (1)" + "\r\n";
            txtListNodos.Text += "i (0)" + "\r\n";
            txtListNodos.Text += "j (0)" + "\r\n";
            txtListNodos.Text += "k (0)" + "\r\n";

            btnAAst.Enabled = false;
            btnAnchura.Enabled = false;
            btnReiniBus.Enabled = false;

            txtRelaciones.Text += grafo.Matriz();
        }

        private void btnReiniBus_Click(object sender, EventArgs e)
        {
            ReiniciarBusqueda();
        }

        public void ReiniciarBusqueda()
        {
            grafo.limpiar2();
            txtIniBus.Text = "";
            txtIniBus.ReadOnly = false;
            txtMeta.Text = "";
            txtMeta.ReadOnly = false;
            txtCamMin.Text = "";
            txtAnchura.Text = "";
            txtAAst.Text = "";
            txtListNodos.Text = "";
            btnAAst.Enabled = false;
            btnAnchura.Enabled = false;
            btnReiniBus.Enabled = false;
            txtMeta.ReadOnly = true;
        }
        //boton para gregar los grafos
        private void btnGrafoAdd_Click(object sender, EventArgs e)
        {
            

            if (grafo.buscarNodo(txtEtiqueta.Text) == -1)
            {
                int numero;
                int.TryParse(txtH.Text, out numero);
                grafo.addVertex(txtEtiqueta.Text, numero);
                txtListNodos.Text += txtEtiqueta.Text + " (" + numero + ")" + "\r\n";
            }
            else
            {
                MessageBox.Show("Nodo repetido", "Advertencia");
            }
        }

        //boton para agregar la relacion
        private void btnGrafoRel_Click(Object sender, EventArgs e)
        {
            if (grafo.buscarNodo(txtNodoIni.Text) != -1)
            {
                if (grafo.buscarNodo(txtNodoFin.Text) != -1)
                {
                    if (grafo.buscarNodo(txtNodoFin.Text) != grafo.buscarNodo(txtNodoIni.Text))
                    {
                        int peso;
                        int.TryParse(txtPeso.Text, out peso);
                        grafo.addEdge(txtNodoIni.Text, txtNodoFin.Text, peso);
                        txtRelaciones.Text = String.Empty;
                        txtRelaciones.Text += grafo.Matriz();
                    }
                    else
                        MessageBox.Show("No se permite que el nodo se llame a si mismo", "Advertencia");
                }
                else
                    MessageBox.Show("No se encontro el nodo final", "Advertencia");
            }
            else
                MessageBox.Show("No se encontro el nodo inicio", "Advertencia");
        }

        private void txtMeta_KeyUp(object sender, KeyEventArgs e)
        {
            if (e.KeyData == Keys.Enter)
            {
                if (grafo.buscarNodo(txtMeta.Text) != -1)
                {
                    grafo.setMeta(txtMeta.Text);
                    btnReiniBus.Enabled = true;
                    txtMeta.ReadOnly = true;
                    btnAAst.Enabled = true;
                    btnAnchura.Enabled = true;
                }
                else
                    MessageBox.Show("No se encontro el nodo", "Advertencia");
            }
        }

        private void btnAAst_Click(object sender, EventArgs e)
        {
            txtAAst.Text = "";
            Console.WriteLine( grafo.Aasterico());
            txtAAst.Text += grafo.Aasterico();
            if (txtAnchura.Text != "No hay solucion" && chkEnviar.Checked)
            {
                char[] c = { '-' };
                asteriscoStrNodo = txtAAst.Text.Split(c);
                asteriscoCont = 0;
                timer1.Start();
                btnAAst.Enabled = false;
                btnAnchura.Enabled = false;
            }
        }

        private void btnAnchura_Click(object sender, EventArgs e)
        {
            //grafo.Anchura();
            
            txtAnchura.Text = grafo.Anchura();
            if (txtAnchura.Text != "No hay solucion" && chkEnviar.Checked)
            {
                char[] c = { '-' };
                asteriscoStrNodo = txtAnchura.Text.Split(c);
                asteriscoCont = 0;
                timer1.Start();
                btnAAst.Enabled = false;
                btnAnchura.Enabled = false;
            }
        }

        private void btnGrafoVac_Click(object sender, EventArgs e)
        {
            grafo.vaciarVertex();
            txtRelaciones.Text = "";
            txtListNodos.Text = "";
            ReiniciarBusqueda();
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            if (asteriscoEnc == true)
            {
                if (asteriscoCont + 1 < asteriscoStrNodo.Length && asteriscoEncCont == 0)
                {
                    int dist = 0;

                    int a = grafo.buscarNodo(asteriscoStrNodo[asteriscoCont + 1]);
                    int b = grafo.buscarNodo(asteriscoStrNodo[asteriscoCont]);
                    dist = a - b;
                    Console.WriteLine(dist);
                    asteriscoEnc = false;
                    asteriscoEncCont= dist;
                    asteriscoCont++;
                }
                else
                {
                    if (asteriscoEncCont != 0)
                    {
                        if (asteriscoEncCont >= 0)
                        {
                            valor = 1;
                        }
                        else
                        {
                            valor = 2;
                        }
                        ImpEnv();
                        timer1.Interval = 110;
                        //asteriscoEnc = false;
                        if (asteriscoEncCont > 0)
                            asteriscoEncCont--;
                        else
                            asteriscoEncCont++;
                        asteriscoEnc = false;
                    }
                    else
                    {
                        timer1.Stop();
                        valor = 0;
                        ImpEnv();
                        btnAnchura.Enabled = true;
                        btnAAst.Enabled = true;
                    }
                }
            }
            else 
            {
                if (asteriscoEncCont != 0)
                {
                    asteriscoEnc = true;
                    valor = 0;
                    ImpEnv();
                    timer1.Interval = 350;
                }
                else
                {
                    asteriscoEnc = true;
                    valor = 0;
                    ImpEnv();
                    timer1.Interval = 1500;
                }
            }
        }

        private void Form1_KeyDown(object sender, KeyEventArgs e)
        {
            if (Convert.ToInt32(e.KeyData) == Convert.ToInt32(Keys.Up))
            {
                if (!chkAde.Checked)
                    chkAde.Checked = true;
            }
            else if (Convert.ToInt32(e.KeyData) == Convert.ToInt32(Keys.Down))
            {
                if (!chkAtr.Checked)
                    chkAtr.Checked = true;
            }
            else if (Convert.ToInt32(e.KeyData) == Convert.ToInt32(Keys.Right))
            {
                if (!chkDer.Checked)
                    chkDer.Checked = true;
            }
            else if (Convert.ToInt32(e.KeyData) == Convert.ToInt32(Keys.Left))
            {
                if (!chkIzq.Checked)
                    chkIzq.Checked = true;
            }
        }
        private void Form1_KeyUp(object sender, KeyEventArgs e)
        {
            if (Convert.ToInt32(e.KeyData) == Convert.ToInt32(Keys.Up))
            {
                if (chkAde.Checked)
                    chkAde.Checked = false;
            }
            else if (Convert.ToInt32(e.KeyData) == Convert.ToInt32(Keys.Down))
            {
                if (chkAtr.Checked)
                    chkAtr.Checked = false;
            }
            else if (Convert.ToInt32(e.KeyData) == Convert.ToInt32(Keys.Right))
            {
                if (chkDer.Checked)
                    chkDer.Checked = false;
            }
            else if (Convert.ToInt32(e.KeyData) == Convert.ToInt32(Keys.Left))
            {
                if (chkIzq.Checked)
                    chkIzq.Checked = false;
            }
        }
    }
}
