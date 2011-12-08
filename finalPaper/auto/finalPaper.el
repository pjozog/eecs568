(TeX-add-style-hook "finalPaper"
 (lambda ()
    (LaTeX-add-bibliographies
     "IEEEabrv"
     "references")
    (LaTeX-add-labels
     "sec:introduction"
     "sec:parrotar.drone"
     "sec:incrementalsmoothingandmapping"
     "sub:givensrotations"
     "sub:variablereordering"
     "fig:images/reorderResult32Resized"
     "fig:images/reorderResult33Resized"
     "fig:reorder"
     "sec:tagcovarianceprojection"
     "sec:experiments"
     "sub:simulation"
     "fig:stepTime"
     "sub:quadrotor"
     "sec:conclusion")
    (TeX-run-style-hooks
     "minted"
     "extarrows"
     "subfigure"
     "epstopdf"
     "comment"
     "amssymb"
     "amsmath"
     "graphicx"
     "parskip"
     "parfill"
     "geometry"
     "latex2e"
     "IEEEtran10"
     "IEEEtran"
     "conference")))

