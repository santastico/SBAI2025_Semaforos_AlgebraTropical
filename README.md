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
