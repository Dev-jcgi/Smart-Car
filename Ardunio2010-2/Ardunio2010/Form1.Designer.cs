namespace Ardunio2010
{
    partial class Form1
    {
        /// <summary>
        /// Variable del diseñador requerida.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Limpiar los recursos que se estén utilizando.
        /// </summary>
        /// <param name="disposing">true si los recursos administrados se deben eliminar; false en caso contrario, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Código generado por el Diseñador de Windows Forms

        /// <summary>
        /// Método necesario para admitir el Diseñador. No se puede modificar
        /// el contenido del método con el editor de código.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.chkAde = new System.Windows.Forms.CheckBox();
            this.chkAtr = new System.Windows.Forms.CheckBox();
            this.chkIzq = new System.Windows.Forms.CheckBox();
            this.chkDer = new System.Windows.Forms.CheckBox();
            this.textBox1 = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.pnlCheck = new System.Windows.Forms.Panel();
            this.textBox2 = new System.Windows.Forms.TextBox();
            this.serialPort1 = new System.IO.Ports.SerialPort(this.components);
            this.cbox = new System.Windows.Forms.ComboBox();
            this.label2 = new System.Windows.Forms.Label();
            this.chkEnviar = new System.Windows.Forms.CheckBox();
            this.inputText = new System.Windows.Forms.TextBox();
            this.textBox4 = new System.Windows.Forms.TextBox();
            this.label3 = new System.Windows.Forms.Label();
            this.tabControl1 = new System.Windows.Forms.TabControl();
            this.tabPage1 = new System.Windows.Forms.TabPage();
            this.tabPage2 = new System.Windows.Forms.TabPage();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.btnAAst = new System.Windows.Forms.Button();
            this.btnAnchura = new System.Windows.Forms.Button();
            this.txtAAst = new System.Windows.Forms.TextBox();
            this.txtAnchura = new System.Windows.Forms.TextBox();
            this.label14 = new System.Windows.Forms.Label();
            this.label13 = new System.Windows.Forms.Label();
            this.btnReiniBus = new System.Windows.Forms.Button();
            this.txtMeta = new System.Windows.Forms.TextBox();
            this.label12 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.txtCamMin = new System.Windows.Forms.TextBox();
            this.txtIniBus = new System.Windows.Forms.TextBox();
            this.label5 = new System.Windows.Forms.Label();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.txtNodoIni = new System.Windows.Forms.TextBox();
            this.btnGrafoDef = new System.Windows.Forms.Button();
            this.txtRelaciones = new System.Windows.Forms.TextBox();
            this.btnGrafoVac = new System.Windows.Forms.Button();
            this.txtNodoFin = new System.Windows.Forms.TextBox();
            this.btnGrafoRel = new System.Windows.Forms.Button();
            this.label9 = new System.Windows.Forms.Label();
            this.txtPeso = new System.Windows.Forms.TextBox();
            this.label10 = new System.Windows.Forms.Label();
            this.label11 = new System.Windows.Forms.Label();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.txtEtiqueta = new System.Windows.Forms.TextBox();
            this.label7 = new System.Windows.Forms.Label();
            this.txtListNodos = new System.Windows.Forms.TextBox();
            this.btnGrafoAdd = new System.Windows.Forms.Button();
            this.label8 = new System.Windows.Forms.Label();
            this.txtH = new System.Windows.Forms.TextBox();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.pnlCheck.SuspendLayout();
            this.tabControl1.SuspendLayout();
            this.tabPage1.SuspendLayout();
            this.tabPage2.SuspendLayout();
            this.groupBox3.SuspendLayout();
            this.groupBox2.SuspendLayout();
            this.groupBox1.SuspendLayout();
            this.SuspendLayout();
            // 
            // chkAde
            // 
            this.chkAde.AutoSize = true;
            this.chkAde.Location = new System.Drawing.Point(75, 11);
            this.chkAde.Name = "chkAde";
            this.chkAde.Size = new System.Drawing.Size(68, 17);
            this.chkAde.TabIndex = 0;
            this.chkAde.Text = "Adelante";
            this.chkAde.UseVisualStyleBackColor = true;
            this.chkAde.CheckedChanged += new System.EventHandler(this.chkAde_CheckedChanged);
            // 
            // chkAtr
            // 
            this.chkAtr.AutoSize = true;
            this.chkAtr.Location = new System.Drawing.Point(75, 59);
            this.chkAtr.Name = "chkAtr";
            this.chkAtr.Size = new System.Drawing.Size(50, 17);
            this.chkAtr.TabIndex = 1;
            this.chkAtr.Text = "Atras";
            this.chkAtr.UseVisualStyleBackColor = true;
            this.chkAtr.CheckedChanged += new System.EventHandler(this.chkAtr_CheckedChanged);
            // 
            // chkIzq
            // 
            this.chkIzq.AutoSize = true;
            this.chkIzq.Location = new System.Drawing.Point(3, 31);
            this.chkIzq.Name = "chkIzq";
            this.chkIzq.Size = new System.Drawing.Size(69, 17);
            this.chkIzq.TabIndex = 2;
            this.chkIzq.Text = "Izquierda";
            this.chkIzq.UseVisualStyleBackColor = true;
            this.chkIzq.CheckedChanged += new System.EventHandler(this.chkIzq_CheckedChanged);
            // 
            // chkDer
            // 
            this.chkDer.AutoSize = true;
            this.chkDer.Location = new System.Drawing.Point(140, 34);
            this.chkDer.Name = "chkDer";
            this.chkDer.Size = new System.Drawing.Size(67, 17);
            this.chkDer.TabIndex = 3;
            this.chkDer.Text = "Derecha";
            this.chkDer.UseVisualStyleBackColor = true;
            this.chkDer.CheckedChanged += new System.EventHandler(this.chkDer_CheckedChanged);
            // 
            // textBox1
            // 
            this.textBox1.Location = new System.Drawing.Point(141, 204);
            this.textBox1.Name = "textBox1";
            this.textBox1.ReadOnly = true;
            this.textBox1.Size = new System.Drawing.Size(33, 20);
            this.textBox1.TabIndex = 4;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(138, 182);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(36, 13);
            this.label1.TabIndex = 5;
            this.label1.Text = "Salida";
            // 
            // pnlCheck
            // 
            this.pnlCheck.Controls.Add(this.chkAtr);
            this.pnlCheck.Controls.Add(this.chkAde);
            this.pnlCheck.Controls.Add(this.chkIzq);
            this.pnlCheck.Controls.Add(this.chkDer);
            this.pnlCheck.Location = new System.Drawing.Point(12, 12);
            this.pnlCheck.Name = "pnlCheck";
            this.pnlCheck.Size = new System.Drawing.Size(214, 90);
            this.pnlCheck.TabIndex = 7;
            // 
            // textBox2
            // 
            this.textBox2.Enabled = false;
            this.textBox2.Location = new System.Drawing.Point(12, 115);
            this.textBox2.Multiline = true;
            this.textBox2.Name = "textBox2";
            this.textBox2.ScrollBars = System.Windows.Forms.ScrollBars.Horizontal;
            this.textBox2.Size = new System.Drawing.Size(107, 128);
            this.textBox2.TabIndex = 8;
            this.textBox2.Text = "0-Nada\r\n1-Adelante\r\n2-Abajo\r\n4-Derecha\r\n8-Izquierda\r\n5-Adelante,Derecha\r\n9-Adelan" +
    "te,Izquierda\r\n6-Abajo,Derecha\r\nA-AbajoIzquierda";
            // 
            // cbox
            // 
            this.cbox.FormattingEnabled = true;
            this.cbox.Location = new System.Drawing.Point(141, 135);
            this.cbox.Name = "cbox";
            this.cbox.Size = new System.Drawing.Size(107, 21);
            this.cbox.TabIndex = 9;
            this.cbox.SelectedIndexChanged += new System.EventHandler(this.cbox_SelectedIndexChanged);
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(138, 115);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(31, 13);
            this.label2.TabIndex = 10;
            this.label2.Text = "COM";
            // 
            // chkEnviar
            // 
            this.chkEnviar.AutoSize = true;
            this.chkEnviar.Enabled = false;
            this.chkEnviar.Location = new System.Drawing.Point(141, 162);
            this.chkEnviar.Name = "chkEnviar";
            this.chkEnviar.Size = new System.Drawing.Size(87, 17);
            this.chkEnviar.TabIndex = 11;
            this.chkEnviar.Text = "Enviar Datos";
            this.chkEnviar.UseVisualStyleBackColor = true;
            this.chkEnviar.CheckedChanged += new System.EventHandler(this.chkEnviar_CheckedChanged);
            // 
            // inputText
            // 
            this.inputText.AutoCompleteCustomSource.AddRange(new string[] {
            "UP",
            "DOWN",
            "LEFT",
            "RIGHT",
            "STOP",
            "NO UP",
            "NO DOWN",
            "NO LEFT",
            "NO RIGHT"});
            this.inputText.AutoCompleteMode = System.Windows.Forms.AutoCompleteMode.Suggest;
            this.inputText.AutoCompleteSource = System.Windows.Forms.AutoCompleteSource.CustomSource;
            this.inputText.CharacterCasing = System.Windows.Forms.CharacterCasing.Upper;
            this.inputText.Location = new System.Drawing.Point(12, 35);
            this.inputText.Multiline = true;
            this.inputText.Name = "inputText";
            this.inputText.Size = new System.Drawing.Size(193, 20);
            this.inputText.TabIndex = 12;
            this.inputText.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.inputText_KeyPress);
            // 
            // textBox4
            // 
            this.textBox4.Enabled = false;
            this.textBox4.Location = new System.Drawing.Point(12, 62);
            this.textBox4.Multiline = true;
            this.textBox4.Name = "textBox4";
            this.textBox4.ScrollBars = System.Windows.Forms.ScrollBars.Horizontal;
            this.textBox4.Size = new System.Drawing.Size(193, 128);
            this.textBox4.TabIndex = 14;
            this.textBox4.Text = "UP\r\nDOWN\r\nLEFT\r\nRIGHT\r\nSTOP\r\nNO UP\r\nNO DOWN\r\nNO LEFT\r\nNO RIGHT";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(9, 4);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(142, 13);
            this.label3.TabIndex = 18;
            this.label3.Text = "Controlar mediante cadenas:";
            // 
            // tabControl1
            // 
            this.tabControl1.Controls.Add(this.tabPage1);
            this.tabControl1.Controls.Add(this.tabPage2);
            this.tabControl1.Location = new System.Drawing.Point(301, 12);
            this.tabControl1.Name = "tabControl1";
            this.tabControl1.SelectedIndex = 0;
            this.tabControl1.Size = new System.Drawing.Size(502, 377);
            this.tabControl1.TabIndex = 19;
            // 
            // tabPage1
            // 
            this.tabPage1.Controls.Add(this.label3);
            this.tabPage1.Controls.Add(this.textBox4);
            this.tabPage1.Controls.Add(this.inputText);
            this.tabPage1.Location = new System.Drawing.Point(4, 22);
            this.tabPage1.Name = "tabPage1";
            this.tabPage1.Padding = new System.Windows.Forms.Padding(3);
            this.tabPage1.Size = new System.Drawing.Size(494, 351);
            this.tabPage1.TabIndex = 0;
            this.tabPage1.Text = "Automata";
            this.tabPage1.UseVisualStyleBackColor = true;
            // 
            // tabPage2
            // 
            this.tabPage2.AutoScroll = true;
            this.tabPage2.Controls.Add(this.groupBox3);
            this.tabPage2.Controls.Add(this.groupBox2);
            this.tabPage2.Controls.Add(this.groupBox1);
            this.tabPage2.Location = new System.Drawing.Point(4, 22);
            this.tabPage2.Name = "tabPage2";
            this.tabPage2.Padding = new System.Windows.Forms.Padding(3);
            this.tabPage2.Size = new System.Drawing.Size(494, 351);
            this.tabPage2.TabIndex = 1;
            this.tabPage2.Text = "Grafo";
            this.tabPage2.UseVisualStyleBackColor = true;
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.btnAAst);
            this.groupBox3.Controls.Add(this.btnAnchura);
            this.groupBox3.Controls.Add(this.txtAAst);
            this.groupBox3.Controls.Add(this.txtAnchura);
            this.groupBox3.Controls.Add(this.label14);
            this.groupBox3.Controls.Add(this.label13);
            this.groupBox3.Controls.Add(this.btnReiniBus);
            this.groupBox3.Controls.Add(this.txtMeta);
            this.groupBox3.Controls.Add(this.label12);
            this.groupBox3.Controls.Add(this.label6);
            this.groupBox3.Controls.Add(this.txtCamMin);
            this.groupBox3.Controls.Add(this.txtIniBus);
            this.groupBox3.Controls.Add(this.label5);
            this.groupBox3.Location = new System.Drawing.Point(16, 278);
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.Size = new System.Drawing.Size(424, 244);
            this.groupBox3.TabIndex = 22;
            this.groupBox3.TabStop = false;
            this.groupBox3.Text = "Algoritmos";
            // 
            // btnAAst
            // 
            this.btnAAst.Enabled = false;
            this.btnAAst.Location = new System.Drawing.Point(327, 157);
            this.btnAAst.Name = "btnAAst";
            this.btnAAst.Size = new System.Drawing.Size(61, 23);
            this.btnAAst.TabIndex = 30;
            this.btnAAst.Text = "Buscar";
            this.btnAAst.UseVisualStyleBackColor = true;
            this.btnAAst.Click += new System.EventHandler(this.btnAAst_Click);
            // 
            // btnAnchura
            // 
            this.btnAnchura.Enabled = false;
            this.btnAnchura.Location = new System.Drawing.Point(327, 89);
            this.btnAnchura.Name = "btnAnchura";
            this.btnAnchura.Size = new System.Drawing.Size(61, 23);
            this.btnAnchura.TabIndex = 29;
            this.btnAnchura.Text = "Buscar";
            this.btnAnchura.UseVisualStyleBackColor = true;
            this.btnAnchura.Click += new System.EventHandler(this.btnAnchura_Click);
            // 
            // txtAAst
            // 
            this.txtAAst.Location = new System.Drawing.Point(156, 157);
            this.txtAAst.Name = "txtAAst";
            this.txtAAst.ReadOnly = true;
            this.txtAAst.Size = new System.Drawing.Size(163, 20);
            this.txtAAst.TabIndex = 28;
            // 
            // txtAnchura
            // 
            this.txtAnchura.Location = new System.Drawing.Point(156, 92);
            this.txtAnchura.Name = "txtAnchura";
            this.txtAnchura.ReadOnly = true;
            this.txtAnchura.Size = new System.Drawing.Size(163, 20);
            this.txtAnchura.TabIndex = 27;
            // 
            // label14
            // 
            this.label14.AutoSize = true;
            this.label14.Location = new System.Drawing.Point(153, 129);
            this.label14.Name = "label14";
            this.label14.Size = new System.Drawing.Size(105, 13);
            this.label14.TabIndex = 26;
            this.label14.Text = "Busqueda Heuristica";
            // 
            // label13
            // 
            this.label13.AutoSize = true;
            this.label13.Location = new System.Drawing.Point(153, 75);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(122, 13);
            this.label13.TabIndex = 25;
            this.label13.Text = "Busqueda Menos Saltos";
            // 
            // btnReiniBus
            // 
            this.btnReiniBus.Enabled = false;
            this.btnReiniBus.Location = new System.Drawing.Point(327, 187);
            this.btnReiniBus.Name = "btnReiniBus";
            this.btnReiniBus.Size = new System.Drawing.Size(75, 41);
            this.btnReiniBus.TabIndex = 24;
            this.btnReiniBus.Text = "Reiniciar Busqueda";
            this.btnReiniBus.UseVisualStyleBackColor = true;
            this.btnReiniBus.Click += new System.EventHandler(this.btnReiniBus_Click);
            // 
            // txtMeta
            // 
            this.txtMeta.Location = new System.Drawing.Point(156, 36);
            this.txtMeta.Name = "txtMeta";
            this.txtMeta.ReadOnly = true;
            this.txtMeta.Size = new System.Drawing.Size(51, 20);
            this.txtMeta.TabIndex = 23;
            this.txtMeta.KeyUp += new System.Windows.Forms.KeyEventHandler(this.txtMeta_KeyUp);
            // 
            // label12
            // 
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(153, 20);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(31, 13);
            this.label12.TabIndex = 22;
            this.label12.Text = "Meta";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(8, 75);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(88, 13);
            this.label6.TabIndex = 21;
            this.label6.Text = "Caminos Minimos";
            // 
            // txtCamMin
            // 
            this.txtCamMin.Location = new System.Drawing.Point(6, 103);
            this.txtCamMin.Multiline = true;
            this.txtCamMin.Name = "txtCamMin";
            this.txtCamMin.ReadOnly = true;
            this.txtCamMin.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
            this.txtCamMin.Size = new System.Drawing.Size(100, 125);
            this.txtCamMin.TabIndex = 20;
            // 
            // txtIniBus
            // 
            this.txtIniBus.Location = new System.Drawing.Point(10, 36);
            this.txtIniBus.Name = "txtIniBus";
            this.txtIniBus.Size = new System.Drawing.Size(49, 20);
            this.txtIniBus.TabIndex = 1;
            this.txtIniBus.KeyUp += new System.Windows.Forms.KeyEventHandler(this.txtIniBus_KeyUp);
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(7, 20);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(108, 13);
            this.label5.TabIndex = 0;
            this.label5.Text = "Inicio de la busqueda";
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.txtNodoIni);
            this.groupBox2.Controls.Add(this.btnGrafoDef);
            this.groupBox2.Controls.Add(this.txtRelaciones);
            this.groupBox2.Controls.Add(this.btnGrafoVac);
            this.groupBox2.Controls.Add(this.txtNodoFin);
            this.groupBox2.Controls.Add(this.btnGrafoRel);
            this.groupBox2.Controls.Add(this.label9);
            this.groupBox2.Controls.Add(this.txtPeso);
            this.groupBox2.Controls.Add(this.label10);
            this.groupBox2.Controls.Add(this.label11);
            this.groupBox2.Location = new System.Drawing.Point(140, 12);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(332, 246);
            this.groupBox2.TabIndex = 16;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "Agregar Relaciones";
            // 
            // txtNodoIni
            // 
            this.txtNodoIni.Location = new System.Drawing.Point(6, 40);
            this.txtNodoIni.Name = "txtNodoIni";
            this.txtNodoIni.Size = new System.Drawing.Size(48, 20);
            this.txtNodoIni.TabIndex = 3;
            // 
            // btnGrafoDef
            // 
            this.btnGrafoDef.Location = new System.Drawing.Point(232, 52);
            this.btnGrafoDef.Name = "btnGrafoDef";
            this.btnGrafoDef.Size = new System.Drawing.Size(80, 37);
            this.btnGrafoDef.TabIndex = 17;
            this.btnGrafoDef.Text = "Usar Grafo por defecto";
            this.btnGrafoDef.UseVisualStyleBackColor = true;
            this.btnGrafoDef.Click += new System.EventHandler(this.btnGrafoDef_Click);
            // 
            // txtRelaciones
            // 
            this.txtRelaciones.AcceptsReturn = true;
            this.txtRelaciones.AcceptsTab = true;
            this.txtRelaciones.Location = new System.Drawing.Point(6, 106);
            this.txtRelaciones.Multiline = true;
            this.txtRelaciones.Name = "txtRelaciones";
            this.txtRelaciones.ScrollBars = System.Windows.Forms.ScrollBars.Both;
            this.txtRelaciones.Size = new System.Drawing.Size(315, 125);
            this.txtRelaciones.TabIndex = 20;
            this.txtRelaciones.WordWrap = false;
            // 
            // btnGrafoVac
            // 
            this.btnGrafoVac.Location = new System.Drawing.Point(232, 23);
            this.btnGrafoVac.Name = "btnGrafoVac";
            this.btnGrafoVac.Size = new System.Drawing.Size(80, 23);
            this.btnGrafoVac.TabIndex = 16;
            this.btnGrafoVac.Text = "Vaciar Grafo";
            this.btnGrafoVac.UseVisualStyleBackColor = true;
            this.btnGrafoVac.Click += new System.EventHandler(this.btnGrafoVac_Click);
            // 
            // txtNodoFin
            // 
            this.txtNodoFin.Location = new System.Drawing.Point(82, 39);
            this.txtNodoFin.Name = "txtNodoFin";
            this.txtNodoFin.Size = new System.Drawing.Size(46, 20);
            this.txtNodoFin.TabIndex = 7;
            // 
            // btnGrafoRel
            // 
            this.btnGrafoRel.Location = new System.Drawing.Point(6, 66);
            this.btnGrafoRel.Name = "btnGrafoRel";
            this.btnGrafoRel.Size = new System.Drawing.Size(104, 23);
            this.btnGrafoRel.TabIndex = 13;
            this.btnGrafoRel.Text = "Agregar Relacion";
            this.btnGrafoRel.UseVisualStyleBackColor = true;
            this.btnGrafoRel.Click += new System.EventHandler(this.btnGrafoRel_Click);
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(6, 17);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(63, 13);
            this.label9.TabIndex = 8;
            this.label9.Text = "Nodo Inicial";
            // 
            // txtPeso
            // 
            this.txtPeso.Location = new System.Drawing.Point(157, 39);
            this.txtPeso.Name = "txtPeso";
            this.txtPeso.Size = new System.Drawing.Size(50, 20);
            this.txtPeso.TabIndex = 11;
            this.txtPeso.Text = "1";
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(79, 17);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(58, 13);
            this.label10.TabIndex = 9;
            this.label10.Text = "Nodo Final";
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(154, 17);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(31, 13);
            this.label11.TabIndex = 10;
            this.label11.Text = "Peso";
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.txtEtiqueta);
            this.groupBox1.Controls.Add(this.label7);
            this.groupBox1.Controls.Add(this.txtListNodos);
            this.groupBox1.Controls.Add(this.btnGrafoAdd);
            this.groupBox1.Controls.Add(this.label8);
            this.groupBox1.Controls.Add(this.txtH);
            this.groupBox1.Location = new System.Drawing.Point(6, 12);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(119, 246);
            this.groupBox1.TabIndex = 15;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Agregar Nodo";
            // 
            // txtEtiqueta
            // 
            this.txtEtiqueta.Location = new System.Drawing.Point(10, 39);
            this.txtEtiqueta.Name = "txtEtiqueta";
            this.txtEtiqueta.Size = new System.Drawing.Size(46, 20);
            this.txtEtiqueta.TabIndex = 1;
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(10, 17);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(46, 13);
            this.label7.TabIndex = 4;
            this.label7.Text = "Etiqueta";
            // 
            // txtListNodos
            // 
            this.txtListNodos.Location = new System.Drawing.Point(10, 106);
            this.txtListNodos.Multiline = true;
            this.txtListNodos.Name = "txtListNodos";
            this.txtListNodos.ReadOnly = true;
            this.txtListNodos.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
            this.txtListNodos.Size = new System.Drawing.Size(100, 125);
            this.txtListNodos.TabIndex = 19;
            // 
            // btnGrafoAdd
            // 
            this.btnGrafoAdd.Location = new System.Drawing.Point(10, 66);
            this.btnGrafoAdd.Name = "btnGrafoAdd";
            this.btnGrafoAdd.Size = new System.Drawing.Size(75, 23);
            this.btnGrafoAdd.TabIndex = 12;
            this.btnGrafoAdd.Text = "Agregar nodo";
            this.btnGrafoAdd.UseVisualStyleBackColor = true;
            this.btnGrafoAdd.Click += new System.EventHandler(this.btnGrafoAdd_Click);
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(71, 17);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(25, 13);
            this.label8.TabIndex = 5;
            this.label8.Text = "h(n)";
            // 
            // txtH
            // 
            this.txtH.Location = new System.Drawing.Point(74, 40);
            this.txtH.Name = "txtH";
            this.txtH.Size = new System.Drawing.Size(38, 20);
            this.txtH.TabIndex = 6;
            this.txtH.Text = "0";
            // 
            // timer1
            // 
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(863, 444);
            this.Controls.Add(this.tabControl1);
            this.Controls.Add(this.textBox2);
            this.Controls.Add(this.pnlCheck);
            this.Controls.Add(this.chkEnviar);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.cbox);
            this.Controls.Add(this.textBox1);
            this.Controls.Add(this.label1);
            this.KeyPreview = true;
            this.Name = "Form1";
            this.Text = "Control";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.Form1_FormClosing);
            this.KeyDown += new System.Windows.Forms.KeyEventHandler(this.Form1_KeyDown);
            this.KeyUp += new System.Windows.Forms.KeyEventHandler(this.Form1_KeyUp);
            this.pnlCheck.ResumeLayout(false);
            this.pnlCheck.PerformLayout();
            this.tabControl1.ResumeLayout(false);
            this.tabPage1.ResumeLayout(false);
            this.tabPage1.PerformLayout();
            this.tabPage2.ResumeLayout(false);
            this.groupBox3.ResumeLayout(false);
            this.groupBox3.PerformLayout();
            this.groupBox2.ResumeLayout(false);
            this.groupBox2.PerformLayout();
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.CheckBox chkAde;
        private System.Windows.Forms.CheckBox chkAtr;
        private System.Windows.Forms.CheckBox chkIzq;
        private System.Windows.Forms.CheckBox chkDer;
        private System.Windows.Forms.TextBox textBox1;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Panel pnlCheck;
        private System.Windows.Forms.TextBox textBox2;
        private System.IO.Ports.SerialPort serialPort1;
        private System.Windows.Forms.ComboBox cbox;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.CheckBox chkEnviar;
        private System.Windows.Forms.TextBox inputText;
        private System.Windows.Forms.TextBox textBox4;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.TabControl tabControl1;
        private System.Windows.Forms.TabPage tabPage1;
        private System.Windows.Forms.TabPage tabPage2;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.Button btnAAst;
        private System.Windows.Forms.Button btnAnchura;
        private System.Windows.Forms.TextBox txtAAst;
        private System.Windows.Forms.TextBox txtAnchura;
        private System.Windows.Forms.Label label14;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.Button btnReiniBus;
        private System.Windows.Forms.TextBox txtMeta;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.TextBox txtCamMin;
        private System.Windows.Forms.TextBox txtIniBus;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.TextBox txtNodoIni;
        private System.Windows.Forms.Button btnGrafoDef;
        private System.Windows.Forms.TextBox txtRelaciones;
        private System.Windows.Forms.Button btnGrafoVac;
        private System.Windows.Forms.TextBox txtNodoFin;
        private System.Windows.Forms.Button btnGrafoRel;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.TextBox txtPeso;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.TextBox txtEtiqueta;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.TextBox txtListNodos;
        private System.Windows.Forms.Button btnGrafoAdd;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.TextBox txtH;
        private System.Windows.Forms.Timer timer1;
    }
}

