#!/bin/sh
set -x
DIR=`dirname $0` &&
cd $DIR &&
#--- Compilation LaTex
rm -f acan2515.pdf ref.* acan2515.ilg acan2515.ind &&
rm -f acan2515.aux acan2515.idx acan2515.lof acan2515.lot acan2515.toc &&
rm -f acan2515.log acan2515.out acan2515.synctex.gz &&
#--- First pass
PDF_LATEX=`which xelatex` &&
#MAKE_INDEX=`which makeindex` &&
$PDF_LATEX --file-line-error --shell-escape acan2515.tex &&
touch ref.idx &&
touch ref.lof &&
touch ref.lot &&
touch ref.toc &&
iteration=0 &&
while [ `cmp -s ref.toc acan2515.toc ; echo $?` -ne 0 ]
#    || [ `cmp -s ref.lot acan2515.lot ; echo $?` -ne 0 ] \
#    || [ `cmp -s ref.toc acan2515.toc ; echo $?` -ne 0 ] \
#    || [ `cmp -s ref.idx acan2515.idx ; echo $?` -ne 0 ]
do
#  cp acan2515.idx ref.idx &&
#  cp acan2515.lof ref.lof &&
#  cp acan2515.lot ref.lot &&
  cp acan2515.toc ref.toc &&
#  $MAKE_INDEX -s $DIR/acan2515-latex-inclusions/style-indexes.ist acan2515.idx &&
  $PDF_LATEX --file-line-error --shell-escape acan2515.tex &&
  iteration=$((iteration+=1))
done &&
rm -f acan2515.aux acan2515.idx acan2515.lof acan2515.lot acan2515.toc &&
rm -f acan2515.log acan2515.ilg acan2515.ind acan2515.out acan2515.synctex.gz &&
echo "---------------- SUCCES $iteration iterations"
