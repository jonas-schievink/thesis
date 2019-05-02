% * select [german] or [english]
% * use option "alpha" for abbreviated citation (instead of numbers)
% * option "draft" is available, too
\ProvidesClass{thesis_KBS}

\LoadClass[11pt,a4paper,twoside,openright]{report}

\newcommand{\setdraft}{}
\DeclareOption{draft}{\renewcommand{\setdraft}{draft}}

\newcounter{sprache}
\DeclareOption{german}{
  \setcounter{sprache}{1}
  \def\@dept{Institut f\"ur Informatik\\AG Wissensbasierte Systeme}
  \def\@type{Masterarbeit}
}
\DeclareOption{english}{
  \setcounter{sprache}{2}
  \def\@dept{Institute for Computer Science\\Knowledge-Based Systems}
  \def\@type{Master's Thesis}
}

\newcounter{bstyle}
\setcounter{bstyle}{1}
\DeclareOption{alpha}{
  \setcounter{bstyle}{2}
}

\ProcessOptions

\RequirePackage{ifthen}

\ifthenelse{\value{sprache} = 1}{
  \RequirePackage[ngerman]{babel}
  \RequirePackage[T1]{fontenc}
  \RequirePackage{bibgerm} % needed when using {gerplain} style

  \ifthenelse{\value{bstyle} = 1}{
    %\bibliographystyle{gerplain} % setzt Vornamen inkonsistent !!!!!!!
    \bibliographystyle{plaindin} 
  }{
    % \bibliographystyle{geralpha} % dito
    \bibliographystyle{alphadin}
  }   
}{
  \RequirePackage[english]{babel}
  \ifthenelse{\value{bstyle} = 1}{
    \bibliographystyle{plain}
  }{
    \bibliographystyle{alpha}
  }   
}


\RequirePackage{fancyhdr}
\RequirePackage[\setdraft]{graphicx}
\RequirePackage{bbm}
\RequirePackage{amsmath}
\RequirePackage{amssymb}
\RequirePackage{fancyhdr}
\RequirePackage{url}
\RequirePackage{cite}


% Option "openright" läßt jedes Kapitel auf einer ungeraden Seite starten;
% Problem: ggf. eingefügte Leerseiten haben noch Kopf- und Fußzeilen!!!
% => Lösung:
  \def\cleardoublepage{\clearpage\if@twoside \ifodd\c@page\else
  \hbox{}
  \thispagestyle{empty}
  \newpage
  \if@twocolumn\hbox{}\newpage\fi\fi\fi}

% hilft gegen "nur Grafik"-Seiten:
\renewcommand{\floatpagefraction}{.9} % Default: .5
\renewcommand{\textfraction}{.15}     % Default: .2
\renewcommand{\arraystretch}{1.2}

\headwidth160mm \lhead[\thepage]{\rightmark}
\chead{} \rhead[\leftmark]{\thepage}
\cfoot{\textsc{\@title}}
\renewcommand{\headrulewidth}{0.4pt}
\renewcommand{\footrulewidth}{0.4pt}

% Textspiegel
\usepackage{typearea}
\areaset[0.75cm]{16cm}{21cm}
\addtolength{\topmargin}{1cm}
\RequirePackage[bf,small]{caption}


\renewcommand{\O}{{\cal O}}
\renewcommand{\leadsto}{\rightsquigarrow}
\newcommand{\V}[1]{\text{\boldmath $#1$}}    % Format for "Vector"
\newcommand{\M}[1]{\V{#1}}                   % Format for "Matrix"

\newcommand{\R}{\mathbbm{R}}
\newcommand{\N}{\mathbbm{N}}
\newcommand{\C}{\mathbbm{C}}
\newcommand{\1}{\mathbbm{1}}                 % Einheitsmatrix


% erstellt Liste, bei der alle Eintr"age um <Beispieltext>-Breite einger"uckt sind!
% Parameter: {<Beispieltext>}
\newenvironment{liste}[2][\rm]{\begin{list}{}{\settowidth{\labelwidth}{{#1#2}}
  \setlength{\leftmargin}{\labelwidth}\addtolength{\leftmargin}{\labelsep}
  \addtolength{\leftmargin}{3ex}%
  \setlength{\parsep}{.5ex plus0.2ex minus 0.2ex}%
  \setlength{\itemsep}{1ex}%
  \renewcommand{\makelabel}[1]{{#1##1\hfill}}}}
  {\end{list}}%
%%%%%%%
% liste + "Doppelpunkt" (notwendig, damit Doppenpunkt außerhalb des Stilparameters #1)
\newenvironment{listeDP}[2][\rm]{\begin{list}{}{\settowidth{\labelwidth}{{#1#2}:}
  \setlength{\leftmargin}{\labelwidth}\addtolength{\leftmargin}{\labelsep}
  \addtolength{\leftmargin}{3ex}%
  \setlength{\parsep}{.5ex plus0.2ex minus 0.2ex}%
  \setlength{\itemsep}{1ex}%
  \renewcommand{\makelabel}[1]{{#1##1}:\hfill}}}
  {\end{list}}%
% erstellt BULLET-Liste, bei der alle Eintr"age um <Beispieltext>-Breite einger"uckt sind!
% Parameter: {<Beispieltext>}
\newenvironment{Liste}[2][\rm]{\begin{list}{}{\settowidth{\labelwidth}{\:$\bullet$\: #1#2}
  \setlength{\leftmargin}{\labelwidth}\addtolength{\leftmargin}{\labelsep}
  \addtolength{\leftmargin}{3ex}%
  \setlength{\parsep}{.5ex plus0.2ex minus 0.2ex}%
  \setlength{\itemsep}{1ex}%
  \renewcommand{\makelabel}[1]{\:$\bullet$\, #1##1\hfill}}}
  {\end{list}}%
\newcommand{\grad}{\ensuremath{^{\circ}}}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

\ifthenelse{\value{sprache} = 1}{
  \usepackage{parskip}
  %\setlength{\parskip}{5pt plus 2pt minus 1pt}
  %\setlength{\parindent}{0pt}
}{}

\def\type#1{\gdef\@type{#1}}
\def\dept#1{\gdef\@dept{#1}}
\def\signline#1{\gdef\@signline{#1}}
\def\signcity#1{\gdef\@signcity{#1}}
\def\submitdate#1{\gdef\@submitdate{#1}}
\def\@title{}
\def\@author{}
\def\@signcity{Osnabr\"uck}


\ifthenelse{\value{sprache} = 1}{
\def\@submitdate{\ifcase\the\month\or
  Jannuar\or Februar\or M\"arz\or April\or Mai\or Juni\or
  Juli\or August\or September\or Oktober\or November\or Deuember\fi
  \space \number\the\year}
}{
\def\@submitdate{\ifcase\the\month\or
  January\or February\or March\or April\or May\or June\or
  July\or August\or September\or October\or November\or December\fi
  \space \number\the\year}
}

\def\@signline{\@signcity, \@submitdate}

\def\firstSupervisor#1{\gdef\@firstSupervisor{#1}}
\def\secondSupervisor#1{\gdef\@secondSupervisor{#1}}
\def\email#1{\gdef\@email{#1}}

\newcommand{\generatetitle}{
  \pagenumbering{roman}
  \pagestyle{plain}
  \thispagestyle{empty}
\begin{center}
%
\begin{figure}[htbp]
\begin{center}
\vspace{4mm}
\includegraphics[height=1.5cm]{unilogo}
\end{center}
\end{figure}
\vspace{-4mm}
%
\LARGE{\textsc{\@dept}}

\vfill

\LARGE{\emph{\@type}}

\vspace{8mm}

\huge{\textbf{\@title}}

\vspace{8mm}

\LARGE{\@author}

\vspace{32mm}

\large{\@submitdate}
\vfill

\begin{tabular}{ll}
  \large
  \ifthenelse{\value{sprache} = 1}{Erstgutachter:}{First supervisor:}
  & \large \@firstSupervisor\\
  \large
  \ifthenelse{\value{sprache} = 1}{Zweitgutachter:}{Second supervisor:}
  & \large \@secondSupervisor
\end{tabular}
\end{center}}

\def\closing{ %the clause that you did not cheat
  \ifthenelse{\value{sprache} = 1}{\chapter*{Erkl\"arung}}{\chapter*{Proclamation}}
  \thispagestyle{empty}

\ifthenelse{\value{sprache} = 1}{
  Hiermit versichere ich, dass ich die vorliegende Arbeit
  selbst\"andig verfasst und keine anderen als die angegebenen
  Quellen und Hilfsmittel benutzt sowie Zitate kenntlich gemacht
  habe.
}{
  Hereby I confirm that I wrote this thesis independently and
  that I have not made use of any other resources or means than
  those indicated.
}
  
  \bigskip\bigskip

\begin{flushright}
\@signline
\end{flushright}
}

\newenvironment{prefacesection}[1]
{\thispagestyle{empty}\centerline{\Large{\textbf{#1}}}\medskip}{}



\newcommand{\startTextChapters}{
  \cleardoublepage
  \pagestyle{fancy}
  \pagenumbering{arabic}
}

\typeout{^^J*****************************^^J Using Thesis_KBS class v1.72^^J*****************************^^J}
