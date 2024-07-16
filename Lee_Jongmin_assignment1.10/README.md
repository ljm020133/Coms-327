# Simple Chess Game

## Overview

This is a simple text-based chess game developed for a assignment 1.10. The game is implemented in C and C++ and follows the basic rules of chess. Players can make moves, capture opponent pieces, and play a full game of chess.

In the face of memory issues with my initial Pokemon project, I pivoted towards a CHESS which is more engaging to me. Following semester standards, I implemented a character-based chessboard with move tracking and support for multiple boards. The development involved creating numerous dynamic functions, allowing for a flexible design approach that adapts to evolving ideas without strict specifications. Key features include a character-based chessboard, move tracking, and a flexible design approach. Usage instructions for compiling, running, and setup are provided in below. Despite initial challenges, the project's flexible design sets the stage for continuous improvement and adaptation, with future plans to refine and enhance the chess game. 

## Features

- Standard chess board with 8x8 grid.
- Traditional chess pieces: King, Queen, Rook, Bishop, Knight, and Pawn.
- Turn-based gameplay: Players take turns making moves.
- Validity checks for legal moves according to chess rules.
- Basic check and checkmate conditions.

## How to Play

1. **Setup:**
   - Run the program by make and ./chess
   - The board will be displayed, and you can start game by hit ENTER key.

2. **Making Moves:**
   - Enter your move in the format by moving cursor with W A S D.
   - To select peice to move you can hit ENTER then will that peice will move with your cursor.
   - Then simply hit ENTER again will place to that location.
   - You can use ESC to deselect peice you selected with ENTER key

3. **Game Progress:**
   - Players take turns making moves until the game reaches a checkmate, stalemate, or a draw.
   - The game state is displayed after each move.

4. **End of Game:**
   - The game ends when one player is in checkmate, leading to victory for the opposing player.
   - Other possible end conditions include stalemate, draw by repetition, or draw by insufficient material.


