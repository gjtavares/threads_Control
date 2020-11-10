#include <iostream>
#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <string>
#include <sstream>
#include <thread>
#include <chrono>
#include <math.h>
#include <mutex>
#include <future>
#include <fstream>
#include <iomanip>

#define PORT 8080

using namespace std;

///Variaveis globais compartilhadas pelas threas process_thread e softPLC_thread

float qin = 0; //qin(t) -- Valor recebido do controlador
float Altura_do_liquido = 0;

std::mutex qin_e_altura_mutex; // Protege as variaveis Altura_do_liquido e qin

std::string GetLineFromCin()
{
    std::string line;
    std::getline(std::cin, line);
    return line;
}


/// Equação diferencial "dh(t)/dt = (qin(t) - qout(t))/(pi*(r0 + alpha*h(t))^2)"
///qout(t) = cv*sqrt(h(t))
///alpha = (r1-r0)/H

float dhdt(float t, float h, float VazaoDeEntrada)
{
    const float  pi = 3.14159265358979f;
    const float cv = 1.0; //Coeficiente de Descarga do tanque
    const float AlturaTanque = 100.0; //H
    const float RaioInferior = 10.0; //r0
    const float RaioSuperior = 18.0; //r1
    float alpha;
    alpha = (RaioSuperior-RaioInferior)/AlturaTanque;

    return((VazaoDeEntrada - (cv*sqrt(h)))/(pi*pow((RaioInferior+alpha*h),2)));
}

///Metodo Runge-Kutta

float rungeKutta(float h0, float VazaoDeEntrada)
{

    float k1, k2, k3, k4;
    float step = 0.005; //Tamanho do passo
    float hn = h0;
    float tn = 0.0;


    // Interação acontece pelo tempo definido

    for (int i=1; i<=20; i++) // 20 interações simulando 100ms do processo
    {
        // Aplica Runge Kuntta para encontrar o proximo valor de h

        k1 = step*dhdt(tn, hn, VazaoDeEntrada);
        k2 = step*dhdt(tn + 0.5*step, hn + 0.5*k1, VazaoDeEntrada);
        k3 = step*dhdt(tn + 0.5*step, hn + 0.5*k2, VazaoDeEntrada);
        k4 = step*dhdt(tn + step, hn + k3,VazaoDeEntrada);

        // Atualiza o valor de h

        hn = hn + (1.0/6.0)*(k1 + 2*k2 + 2*k3 + k4);;

        // Atualiza o valor de t

        tn = tn + step;
    }

    if (hn<0.0)
    {
        return 0.0;
    }
    else
    {
        return hn;
    }

}

///Controlador ON OFF

float Controlador_on_off(float referencia, float h0)
{

    float Vazao_Maxima_Entrada = 30.0; // Vazão para a valvula de entrada totalmente aberta
    float Vazao_entrada;
    if(h0<= 98 )  // Limite de segurança
    {
        if(h0>referencia)
        {
            Vazao_entrada = 0;
        }
        else
        {
            Vazao_entrada = Vazao_Maxima_Entrada;
        }
        return Vazao_entrada;
    }
    else
    {
        Vazao_entrada = 0;
        return Vazao_entrada;
    }
}

///Emula sistema supervisorio

void synoptic_process()
{
    std::this_thread::sleep_for(std::chrono::seconds(1));
    int sock = 0, valread;
    struct sockaddr_in serv_addr;
    char buffer[1024] = {0};
    int i=0;
    string Altura_ref = "8";
    ofstream out;
    out.open("historiador.txt"); // Aquivo no qual as informações são registradas
    auto future = std::async(std::launch::async, GetLineFromCin);

    //Cria socket para comunicação

    sock = socket(AF_INET, SOCK_STREAM, 0);

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr);


    connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr));


    while(true)
    {
        //Resposável por ler o valor de href do teclado
        if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
        {
            auto line = future.get();

            future = std::async(std::launch::async, GetLineFromCin);

            Altura_ref = line + " ";
        }


        int n = Altura_ref.length();
        char mensagem[n+1];
        strcpy(mensagem, Altura_ref.c_str());
        send(sock, mensagem, strlen(mensagem), 0 );
        valread = read( sock, buffer, 1024);
        if(i==10)
        {
            int n = strlen(buffer);
            buffer[n] = '\0';
            string buf = buffer;
            string imprime = buf + " | href= " + Altura_ref + "m";
            cout << imprime << endl << endl;
            out << imprime << endl;
            i=0;
        }
        i++;
    }
}


/// Função que executa a thread que simula a equação dinamica do tanque

void process_thread()
{
    int espera;
    while(true)
    {
        try
        {
            std::lock_guard<std::mutex> lock(qin_e_altura_mutex); //Tenta adquirir a posse mutex para entrar na seção critica
            Altura_do_liquido = rungeKutta(Altura_do_liquido, qin); //Seção critica Acessa variaveis compartilhadas qin e Altura_do_liquido
            espera = 0;
        }
        catch (std::logic_error&) //Caso não consiga adquirir a posse do mutex
        {

            espera = 1;
        }

        if(espera==0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); //Espera 100ms, período de simulação
        }

    }

}

///Função que executa a thread do processo simulado

void softPLC_thread()
{
    int server_fd, new_socket, valread;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    char buffer[1024] = {0};
    int espera;
    float referencia = 8; //Altura de referencia. Valor recebido do sistema supervisorio
    stringstream altura;
    stringstream vazao_entrada;
    stringstream vazao_saida;
    string valor_atura;
    string valor_qin;
    string valor_qout;
    string msg;

    // Cria socket para comunicação

    server_fd = socket(AF_INET, SOCK_STREAM, 0);

    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));


    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons( PORT );

    bind(server_fd, (struct sockaddr *)&address, sizeof(address));

    listen(server_fd, 3);

    new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen);

    while(true)
    {
        try
        {
            std::lock_guard<std::mutex> lock(qin_e_altura_mutex); //Tenta adquirir a posse mutex para entrar na seção critica
            qin = Controlador_on_off(referencia, Altura_do_liquido); //Seção critica Acessa variaveis compartilhadas qin e Altura_do_liquido
            altura.str("");
            vazao_entrada.str("");
            vazao_saida.str("");
            altura << setprecision(4) << Altura_do_liquido;
            vazao_entrada << setprecision(3) << qin;
            vazao_saida << setprecision(4) << sqrt(Altura_do_liquido);
            valor_atura = altura.str();
            valor_qin = vazao_entrada.str();
            valor_qout = vazao_saida.str();
            msg = "h= " + valor_atura + "m "+ "|" + " qin= " + valor_qin + " m3/s " + "|" + " qout= " + valor_qout + " m3/s" + " ";
            espera = 0;
        }
        catch (std::logic_error&) //Caso não consiga adquirir a posse do mutex
        {

            espera = 1;
        }

        if(espera==0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200)); //Espera por 200ms, períoda da tarefa de controle

            int n = msg.length();
            char mensagem[n+1];
            strcpy(mensagem, msg.c_str());
            valread = read( new_socket, buffer, 1024);
            n = strlen(buffer);
            buffer[n] = '\0';
            double ref_h = atof(buffer);
            referencia = float(ref_h);
            send(new_socket, mensagem, strlen(mensagem), 0 );
        }

    }

}

int main()
{
    pid_t pid;

    pid = fork();

    if(pid < 0)
    {
        cout << "Fork Failed" << endl;
        return 1;
    }
    if(pid==0)
    {
        synoptic_process(); //Cria um processo filho que emula o supervisorio
    }
    else
    {
        thread t1(process_thread); //Cria a thread que emula o processo
        thread t2(softPLC_thread); //Cria a thread que efutua o controle do processo simulado

        t1.join();
        t2.join();
    }

    return 0;
}

