# Sistemas a Eventos Discretos Tropicais: Comunicação e Sincronização de Semáforos

## Autores

- **G. S. Pereira** – LOPAC, Departamento de Engenharia Elétrica, Escola de Engenharia, UFMG  
- **G. F. de Oliveira** – LOPAC, Departamento de Engenharia Elétrica, Escola de Engenharia, UFMG  
- **C. A. Maia** – LOPAC, Departamento de Engenharia Elétrica, Escola de Engenharia, UFMG  

---

## Visão geral

Este projeto apresenta a modelagem e simulação de uma rede de semáforos da Avenida do Contorno, em Belo Horizonte (MG), utilizando Grafos de Eventos Temporizados (GET) e Álgebra Tropical (Max-Plus).  
O objetivo é analisar o comportamento dinâmico dos sistemas semafóricos sob diferentes configurações de tempos, relações de cooperação e restrição entre sinais, permitindo estudar efeitos de atrasos e possíveis falhas de operação.  
A simulação é realizada em Scilab/ScicosLab, com geração de gráficos que mostram a evolução temporal das cores (vermelho, amarelo, verde) de cada semáforo.

---

## Estrutura do repositório

```text
SBAI2025---Traffic-Lights/
│
├── Apresentacao_SBAI2025.pdf     # Slides da apresentação no SBAI 2025
├── GIF_animacao_GETs.gif         # Animação dos Grafos de Eventos Temporizados
├── SBAI2025.sce                  # Script principal em Scilab/ScicosLab
├── Simulacao_HTML_SBAI.html      # Página HTML opcional de visualização
├── simul_sbai.svg                # Diagrama SVG da rede de semáforos
├── jquery-3.7.1.slim.min.js      # Biblioteca jQuery para a página HTML
└── README.md                     # Este arquivo
```

---

## Resumo dos arquivos

- **Apresentacao_SBAI2025.pdf**  
  Slides oficiais do trabalho apresentado no SBAI 2025, com introdução, modelagem, estudo de caso na Av. do Contorno e resultados de simulação.

- **GIF_animacao_GETs.gif**  
  Pequena animação ilustrando o funcionamento dos Grafos de Eventos Temporizados usados para representar os semáforos.

- **SBAI2025.sce**  
  Script principal em Scilab/ScicosLab.  
  Lê parâmetros definidos pelo usuário (número de sinais, matrizes de cooperação e restrição, tempos de cada cor e estado inicial) e executa a simulação, produzindo os gráficos de evolução temporal dos sinais.

- **Simulacao_HTML_SBAI.html**  
  Página HTML opcional para visualizar a rede de semáforos e, se desejado, associar resultados da simulação a uma representação gráfica no navegador.

- **simul_sbai.svg**  
  Arquivo SVG com o desenho da rede de semáforos (cruzamentos e posições dos sinais), usado pela página HTML.

- **jquery-3.7.1.slim.min.js**  
  Biblioteca JavaScript necessária para a interatividade na página `Simulacao_HTML_SBAI.html`.

---

## Como executar

### 1. Simulação em Scilab/ScicosLab

1. Abra o Scilab/ScicosLab.  
2. Carregue o arquivo `SBAI2025.sce`.  
3. Ajuste, se necessário, os parâmetros internos:
   - número de sinais da rede;  
   - matriz de tempos (vermelho, amarelo, verde) para cada semáforo;  
   - marcação inicial (cor ativa em cada sinal);  
   - matrizes de cooperação e restrição;  
   - número de transições a simular.  
4. Execute o script.  
5. Analise os gráficos gerados para verificar o comportamento temporal dos semáforos, incluindo efeitos de cooperação, restrição, atrasos ou falhas.

### 2. Visualização em navegador (opcional)

1. Deixe `Simulacao_HTML_SBAI.html`, `simul_sbai.svg` e `jquery-3.7.1.slim.min.js` na mesma pasta.  
2. Abra `Simulacao_HTML_SBAI.html` em um navegador moderno (Chrome, Firefox, Edge etc.).  
3. Use a página para visualizar o diagrama da rede e, se configurado, associar a animação aos resultados da simulação.

---

## Agradecimentos

- Departamento de Engenharia Elétrica, Escola de Engenharia, Universidade Federal de Minas Gerais (UFMG)  
- Conselho Nacional de Desenvolvimento Científico e Tecnológico (CNPq)  

---

## Contato

Para dúvidas, sugestões ou colaborações:

- **Gabriel Santos Pereira** – [santospereiragsp@gmail.com](mailto:santospereiragsp@gmail.com)

---

## Licença

Este projeto é disponibilizado para uso acadêmico e de pesquisa.  
Se utilizar o código, modelos ou resultados em publicações, por favor cite o trabalho correspondente apresentado no **SBAI 2025**.

---

**Última atualização:** fevereiro de 2026

