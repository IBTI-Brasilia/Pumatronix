# ITSCAM: Projeto Básico de Usabilidade e Arquitetura

## Pacotes
- C++
- C
- Python
- HTML
- JavaScript

## Objetivo
A motivação da PUMATRONIX no desenvolvimento deste projeto é aumentar a qualidade de seus produtos em software, atualizando a interface gráfica e melhorando a 
composição dos componentes de software. Isso se efetivará através de novas propostas de design visual iterative; bem como a realização de provas de conceito de 
arquiteturas de software. Os cenários de Uso da Solução ITSCAMPRO são:
- Mobilidade Urbana;
- Monitoramento e Segurança;
- Concessionárias de Rodovias.

## Arquitetura
A atual linha de dispositivos de captura de imagem da Pumatronix é a ITSCAM 400. Esse dispositivo possui uma série de maneira de ser integrado à 
solução do cliente. A Pumatronix fornece para esse dispositivo a biblioteca <itscam.dll>, sendo uma implementação em C das principais mensagens de 
configuração e de requisição de vídeo e foto para a linha ITSCAM 400. Além da <itscam.dll>, a linha ITSCAM 400 possui seu protocolo aberto e documentado no 
manual do produto o que permite aos clientes implementarem a sua própria interface de interação com o dispositivo.
Apesar de ser uma dll muito simples, a Pumatronix tem mapeado alguns problemas da versão atual:
- A mesma só possui versão para Windows 32. Isso é um problema pois a cada vez mais os clientes têm buscado soluções Linux e em muitos casos embarcados em 
soluções com processadores ARM.
- O projeto de geração da dll depende de uma versão antiga do Visual Studio. Atualmente todo o ambiente de desenvolvimento e de geração das 
soluções da Pumatronix é Linux. Para as biblioteca e aplicações que rodam em Windows é utilizado o MinGW para fazer a cross compilação.
- Pouco pessoas da equipe tem conhecimento sobre a estrutura da dll. Como essa biblioteca foi a bastante tempo e praticamente não teve melhoria e evolução a 
equipe atual não possui conhecimento do seu processo de geração.
- Leak de Memória. Existe uma falha conhecida de leak de memória que não foi possível identificar a causa.

## Etapas e resultados esperados
### PROPOSTA DE NOVA ARQUITETURA PARA DLL PARA O PRODUTO ITSCAM 600
Para essa etapa, é prevista a apresentação de arquiteturas propostas pelo IBTI para a DLL do produto ITSCAM 600, bem como a validação dos modelos 
através de PoC - Provas de Conceito utilizando parte do conjunto de comandos, priorizado pela equipe de engenharia da Pumatronix. 
A DLL da ITSCAM 600 deve ser compatível com as principais plataformas que os clientes utilizam, sendo as hoje conhecidas:
- Windows 7 ou superior (32 bits)
- Ubuntu 12.04 ou superior (32 bits ou 64 bits)
- ARM A9 (32 bits)
- ARM A53 (64 bits)

A Pumatronix irá fornecer a infraestrutura de geração para multiplataformas que é utilizada nos produtos internos.

### MELHORAR A UX AO UTILIZAR O ITSCAMPRO
O ITSCAMPRO possui um interface que foi desenvolvida em 2014 e que vem ser feito a sua manutenção desde então em uma modernização da interface. 
Utilizar recursos visuais e de usabilidade mais modernos para melhorar a experiência do usuário ao configurar e operar o sistema com Skin personalizável
e i18n (internacionalização).

Compatibilidade com principais navegadores:
- Chrome
- Firefox
- Safari

### ARQUITETURA DE SOFTWARE DA DLL ITSCAM 600
- Ser possível configurar todos os parâmetros disponíveis na ITSCAM 600 através de uma API simples e intuitiva;
- Ser possível configurar a ITSCAM 600 para gerar quadros de vídeo de maneira contínua e registrar uma callback para receber os eventos de novos quadros de vídeo;
- Ser possível configurar a ITSCAM 600 para trigger de movimento e registrar uma callback para receber os eventos de novos quadros de foto;
- Ser possível configurar a ITSCAM 600 para trigger via IO e registrar uma callback para receber os eventos de novos quadros de foto.

### USABILIDADE DE SOFTWARE ITSCAMPRO
- Interface Responsiva: Permitir fazer configurações e ver relatórios tanto e telas grandes como TVs como em celulares;
- CRUD Funcional: Ser possível fazer as operações básicas do CRUD das configurações do ITSCAMPRO;
- Relatório Customizado: Permitir que o usuário possa criar seu próprio relatório com uma query customizada estilo PowerBI;
- Relatórios de Monitoramento Urbano: Disponibilizar os principais relatórios para o mercado de monitoramento urbano: Origem Destino, Tempo médio de percurso, etc;
- Relatórios de Segurança Pública: Relatório utilizados para os módulos de Segurança Pública como Veículos Monitorados e Correlação de passagens;

