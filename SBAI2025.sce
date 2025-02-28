//  Autor: Gabriel S. Pereira
//  Supervisionado por: C. A. Maia
//  Data: 28 de fevereiro de 2024
//  Objetivo: Desenvolver uma matriz X que contenha 
//  as datas de transi��o para um sistema de sem�foros
//  � necess�rio incluir:
// ==> N�mero de sinais
// ==> Restri��es e Coopera��es entre eles
// ==> Tempo para cada cor do sem�foro
// ==> Estado inicial

// Limpa a janela de comandos
clc();

// Cria��o de uma vari�vel para medir o tempo de execu��o
tic();

// N�mero de sinais do sistema

                p = 12;  //ENTRADA

// Matriz de Restri��es
// Se M_rst(i, j) = 1, isso significa que 
// o sinal i n�o pode abrir se o sinal j estiver aberto, ou seja, 
// a segunda transi��o (abertura) do sinal i s� ocorre 
// ap�s a primeira transi��o (fechamento) do sinal j. 
// Al�m disso, a segunda transi��o (abertura) do sinal j s� ocorre
// ap�s a primeira transi��o (fechamento) do sinal i.

M_rst = zeros(p,p);

                // Entrada dos Sem�foros de Tr�nsito
                M_rst(1,5)=1;
                M_rst(5,6)=1;
                M_rst(6,2)=1;

                // Entrada dos Sem�foros de Pedestres
                M_rst(3,7)=1;
                M_rst(6,8)=1; M_rst(6,11)=1;
                M_rst(4,9)=1; 
                M_rst(5,10)=1;
                M_rst(1,12)=1;

// Matriz de Coopera��o
// Se M_cop(i, j) = 1, isso significa que
// o sinal i n�o pode abrir se o sinal j estiver fechado, ou seja,
// a segunda transi��o (abertura) do sinal i s� ocorre
// ap�s a segunda transi��o (abertura) do sinal j.
// Al�m disso, a primeira transi��o (fechamento) do sinal j s� ocorre
// ap�s a primeira transi��o (fechamento) do sinal i.

M_cop = zeros(p,p);

                // Entrada dos Sem�foros de Tr�nsito
                M_cop(6,4)=1;
                M_cop(2,4)=1;

                // Entrada dos Sem�foros de Pedestres

// N�mero de transi��es a serem registradas

                k = 4;  //ENTRADA

//----------------------------------------------------------------------------//
// Inicializa��o da matriz de tempos de atraso
// Linha = Sinal
// Coluna = amarelo | vermelho | verde

T = #(zeros(p, 3)); T(:) = %0;

            //ENTRADA

        T(1:p / 2, 1) = 4;
        T(p / 2 + 1:p, 1) = 5;

        T(:, 2) = [49; 36; 39; 27; 71; 84; 81; 40; 93; 49; 36; 71];
        T(:, 3) = [67; 80; 77; 89; 45; 32; 34; 75; 22; 66; 79; 44];

//----------------------------------------------------------------------------//
// Inicializa��o da matriz de estado inicial
// Linha = Sinal
// Coluna = amarelo | vermelho | verde
// Por exemplo, se M_0(7,3) == 1, ent�o o sinal 7 est� verde

            //ENTRADA

        M_0(:, 1) = [0; 0; 0; 0; 0; 0; 
                    0; 0; 0; 0; 0; 0];
        M_0(:, 2) = [0; 1; 1; 1; 1; 0;  //Sem�foro de tr�nsito em vermelho : 5,6     //t1
                    0; 1; 0; 0; 1; 1];  //Sem�foro de pedestre em vermelho : 1,3,6   //t1
        M_0(:, 3) = [1; 0; 0; 0; 0; 1;  //Sem�foro de tr�nsito em verde   : 1,2,3,4 //t2
                    1; 0; 1; 1; 0; 0];  //Sem�foro de pedestre em verde   : 2,4,5   //t2
                    
// Seleciona os �ndices dos sinais desejados a serem apresentados no terminal
selected_signals = [2,4];

//A L�GICA COME�A AQUI
//----------------------------------------------------------------------------//
//----------------------------------------------------------------------------//
// Inicializa��o das matrizes A_0 e A_1 para a equa��o
// X(K) = A_0 * X(K) + A_1 * X(K-1) + B * U(k)
// X(K) = star(A_0) * A_1 * X(K-1) + star(A_0) * U(K)

A_0 = #(zeros(3 * p, 3 * p)); A_0(:) = %0;
A_1 = #(zeros(3 * p, 3 * p)); A_1(:) = %0;

for i = 1:p,
    if M_0(i, 2) then
        A_0(i * 3 - 2, i * 3) = T(i, 1);
        A_0(i * 3, i * 3 - 1) = T(i, 3);
        A_1(i * 3 - 1, i * 3 - 2) = T(i, 2);                                
    elseif M_0(i, 3) then
        A_0(i * 3 - 2, i * 3) = T(i, 1);
        A_0(i * 3 - 1, i * 3 - 2) = T(i, 2);
        A_1(i * 3, i * 3 - 1) = T(i, 3);                      
    elseif M_0(i, 1) then
        A_0(i * 3 - 1, i * 3 - 2) = T(i, 2);
        A_0(i * 3, i * 3 - 1) = T(i, 3);
        A_1(i * 3 - 2, i * 3) = T(i, 1);  
    end,
end;

//----------------------------------------------------------------------------//
// Inicializa��o das matrizes F_0 e F_1 para a equa��o 
// X(K) = A_0 * X(K) + A_1 * X(K-1) + B * U(k)
// X(K) = star(A_0) * A_1 * X(K-1) + star(A_0) * U(K)
// U(k) = F_0 * X(K) + F_1 * X(K-1)   // F_0 e F_1 s�o os controladores das matrizes de restri��o e coopera��o

F_0 = #(zeros(3 * p, 3 * p)); F_0(:) = %0;
F_1 = #(zeros(3 * p, 3 * p)); F_1 (:) = %0;

for i = 1:p,
    for j = 1:p,
        if ( ( M_rst(i, j) & M_0(i, 2) & M_0(j, 3) ) | ( M_rst(i, j) & M_0(i, 3) & M_0(j, 2) ) ) then
            F_0(i * 3 - 1, j * 3 - 2) = %1;
            F_0(j * 3 - 1, i * 3 - 2) = %1;
        
        elseif (M_rst(i, j) & M_0(i, 2) & M_0(j, 2) ) then
            F_1(i * 3 - 1, j * 3 - 2) = %1;
            F_1(j * 3 - 1, i * 3 - 2) = %1;
        
        elseif (M_cop(i, j) & M_0(i, 3) & M_0(j, 3) ) then 
            F_0(i * 3 - 1, j * 3 - 1) = %1;
            F_0(j * 3 - 2, i * 3 - 2) = %1;
        
        elseif (M_cop(i, j) & M_0(i, 2) & M_0(j, 3) ) then
            F_1(i * 3 - 1, j * 3 - 1) = %1;
            F_1(j * 3 - 2, i * 3 - 2) = %1;
        end,
    end,
end;

//----------------------------------------------------------------------------//
// Podemos encontrar a data K-�sima de qualquer transi��o
// Atrav�s da seguinte f�rmula:
// X(3*p,K) = A * X(3*p,K-1)

// Matriz A (F�rmula)
//A = (star(A_0) + star(F_0)) * A_1 + (star(A_0) + star(F_0)) * F_1; #F�rmula 1
//A = (star(A_0 + F_0)) * A_1 + (star(A_1 +F_0)) * F_1; #F�rmula 2

A_2=%eye(3*p,3*p);

A_2=#(A_2);

for j=1:3*p,
    A_2=A_2+(A_0+F_0)^j;
end;

A = A_2 * (A_1 + F_1);

// Matriz X0 (Condi��o Inicial do sistema)
[M, LB] = eigenspace(A);

        // M (Autovetores da matriz A) -> Define uma condi��o inicial
        // LB (Autovalores da matriz A) -> Define o ciclo do sistema
        
[l,c]=size(M);

e = zeros(c, 1);

X0 = M * e; // X0 atualmente cont�m datas negativas

Min_x0 = -1 * min(plustimes(X0));

Min_x0 = #(Min_x0);

X0 = Min_x0 * X0; // X0 corrigido com um deslocamento de Min_x0

X = #(zeros(3 * p, k)); X(:,:) = %0;

X(:,1) = X0;

for i = 2:k,
    X(:, i) = A* X(:, i - 1);
end;

//X(i,j) � a matriz onde i � a transi��o e j � a k-�sima data

//---------------------------Exibi��o|Terminal---------------------------------//
//----------------------------------------------------------------------------//
// Criando um terminal visual e informativo
// Para exibir as datas de disparo K-�sima de um Sem�foro

for i = 1:(p/2)
    tl_disp = 3 * i;
    tl_in_matrix = "Sem�foro " + string(i) + " || ";

    x_disp = zeros(4,k+1);

    for j = 1:k,
        x_disp(1,j+1) = j;
        x_disp(2,j+1) = X(tl_disp-2,j);
        x_disp(3,j+1) = X(tl_disp-1,j);
        x_disp(4,j+1) = X(tl_disp,j);
    end;

    x_disp = string(x_disp);
    x_disp(1,1) = tl_in_matrix;
    x_disp(2,1) = "Vermelho -----> ";
    x_disp(3,1) = "Verde --------> ";
    x_disp(4,1) = "Amarelo ------> ";
    
    // Terminal:
    // Sem�foro sendo analisado
    disp(x_disp);
    
    disp("----------------------------------------------------------------------------------------------------");
    
end;

//----------------------------------------------------------------------------//
// Criando um terminal visual e informativo
// Para exibir as datas de disparo K-�sima de um Sem�foro de Pedestres

for i = 1:(p/2)
    pl_disp = 3 * (i+6);
    pl_in_matrix = "Sinal Pedestre " + string(i) + " || ";

    x_disp_p = zeros(4,k+1);

    for j = 1:k,
        x_disp_p(1,j+1) = j;
        x_disp_p(2,j+1) = X(pl_disp-2,j);
        x_disp_p(3,j+1) = X(pl_disp-1,j);
        x_disp_p(4,j+1) = X(pl_disp,j);
    end;

    x_disp_p = string(x_disp_p);
    x_disp_p(1,1) = pl_in_matrix;
    x_disp_p(2,1) = "Vermelho -----> ";
    x_disp_p(3,1) = "Verde --------> ";
    x_disp_p(4,1) = "Amarelo ------> ";
    
    // Terminal:
    // Sinal de pedestre sendo analisado
    disp(x_disp_p);
    
    disp("----------------------------------------------------------------------------------------------------");
    
end;

// Ciclo do Sistema
disp("Ciclo (em segundos): " + string(LB));

// Tempo de execu��o
disp("Tempo de Execu��o: " + string(toc()));

// Gr�ficos
// Carregar os dados
Data = plustimes(X);

//-------------------------------------------------------------------------%
// N�mero de sinais
num_signals = size(Data, "r") / 3;

// Criar a figura principal
scf(0); // Abrir uma nova janela de figura

// Contador de subplots
subplot_idx = 1;

// Iterar sobre os sinais selecionados
for signal = selected_signals

    // Extrair as transi��es do sinal atual (Vermelho, Verde, Amarelo)
    // "Transitions" armazena os dados K do sinal atual
    transitions = Data((signal - 1) * 3 + 1 : signal * 3, :);

    // Determinar o menor tempo e o estado inicial correspondente para o sinal
    [min_time, min_idx] = min(transitions(:, 1)); // Menor valor e linha correspondente
    if min_idx == 1 then
        current_state = 0.5;    // Linha vermelha: estado inicial � amarelo
    elseif min_idx == 2 then
        current_state = 0;      // Linha verde: estado inicial � vermelho
    else
        current_state = 1;      // Linha amarela: estado inicial � verde
    end

    // Inicializar os arrays de tempo e estado
    times = [0];                // Iniciar no tempo 0
    y_vals = [current_state];   // Estado inicial

    // Ordenar as transi��es por tempo
    // Achatando a matriz
    transitions_flat = [];
    for i = 1:size(transitions, "r")
        transitions_flat = [transitions_flat, transitions(i, :)]; // Adiciona linha a linha
    end

    // Ordenar valores e obter �ndices
    [sorted_times, sorted_idx] = gsort(transitions_flat, "g", "i");

    // Calcular o �ndice da linha correspondente para cada valor ordenado
    originalRows = ceil(sorted_idx / size(transitions, "c"));

    // Iterar sobre os tempos ordenados
    for i = 1:length(sorted_times)
        // Duplicar o tempo atual para manter o estado constante at� a pr�xima transi��o
        times($ + 1) = sorted_times(i);   // Tempo da transi��o atual
        y_vals($ + 1) = y_vals($);        // Repetir o estado atual

        // Atualizar tempo e estado para o pr�ximo passo
        times($ + 1) = sorted_times(i);   // Tempo da transi��o
        if originalRows(i) == 1 then
            current_state = 0;    // Linha vermelha -> amarelo
        elseif originalRows(i) == 2 then
            current_state = 1;    // Linha verde -> vermelho
        elseif originalRows(i) == 3 then
            current_state = 0.5;  // Linha amarela -> verde
        end
        y_vals($ + 1) = current_state;    // Atualizar o estado
    end

    // Adicionar cada gr�fico como um subplot
    subplot(length(selected_signals), 1, subplot_idx); // Organizar subplots em uma �nica coluna
    plot2d(times, y_vals, style = 2); // Usar plot2d para degraus (aproximado)
    xtitle("Tempo (t)", "Estado (y)", ["Sinal " + string(signal)]);
    line_width = 2;
    font_size = 2;
    
    // Incrementar o �ndice do subplot
    subplot_idx = subplot_idx + 1;
end

