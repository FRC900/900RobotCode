set nocompatible
source $VIMRUNTIME/vimrc_example.vim

set vb
set shiftwidth=4
set ts=4
set cindent
set sm

":au BufWrite *.c set expandtab
":au BufWrite *.c retab
":au BufWrite *.h set expandtab
":au BufWrite *.h retab

let c_space_errors=1
set belloff=all

" set the runtime path to include Vundle and initialize
set rtp+=~/.vim/bundle/Vundle.vim
call vundle#begin()
" alternatively, pass a path where Vundle should install plugins
"call vundle#begin('~/some/path/here')

" let Vundle manage Vundle, required
Plugin 'VundleVim/Vundle.vim'

Plugin 'ycm-core/YouCompleteMe'
Plugin 'kgreenek/vim-ros-ycm'

" All of your Plugins must be added before the following line
call vundle#end()            " required
filetype plugin indent on    " required
" To ignore plugin indent changes, instead use:
"filetype plugin on

" Brief help
" :PluginList       - lists configured plugins
" :PluginInstall    - installs plugins; append `!` to update or just :PluginUpdate
" :PluginSearch foo - searches for foo; append `!` to refresh local cache
" :PluginClean      - confirms removal of unused plugins; append `!` to auto-approve removal
"
" see :h vundle for more details or wiki for FAQ
" Put your non-Plugin stuff after this line
let g:ycm_server_python_interpreter = '/usr/bin/python2.7'
