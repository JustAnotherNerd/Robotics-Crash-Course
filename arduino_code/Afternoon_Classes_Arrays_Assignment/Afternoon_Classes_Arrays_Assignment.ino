class Card{
  
  char suit;
  int value;
  
  public:
    Card(char, int);
    
    char getSuit(){
      return suit;
    }
    
    int getValue(){
      return value;
    }
    
};

/* class Deck{
  
  Card cards[52];
  int topCard;

  public:
    Deck(int);
    
    Card getTopCard(){
      if(topCard < 52){
        topCard++;
        return cards[topCard - 1];
      }
    }
    
    void reset(){
      char suits[4] = {'H', 'C', 'S', 'D'};
      for(int i = 0; i < 4; i++){
        for(int j = 0; j < 13; j++){
          Card newCard(suits[i], j + 1);
          cards[13 * i + j] = newCard;
        }
      }
      topCard = 0;
    }

    void shuffle(){
      for(int i = 0; i < 52; i++){
        Card temp = cards[0];
        int randPos = random(i, 52);
        cards[0] = cards[randPos];
        cards[randPos] = temp;
      }
    }

    void printDeck(){
      for(int i = 0; i < 52; i++){
        Serial.print(cards[i].getSuit());
        Serial.println(cards[i].getValue());
      }
    }
};
*/

Card::Card(char s, int v){
  suit = s;
  value = v;
}
/*
Deck::Deck(int tc){
  char suits[4] = {'H', 'C', 'S', 'D'};
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 13; j++){
      Card newCard(suits[i], j + 1);
      cards[13 * i + j] = newCard;
    }
  }
  topCard = tc;
}
*/
void setup() {
  Serial.begin(9600);
/*Deck d(0);
  Card c = d.getTopCard();
  Serial.print(c.getSuit());
  Serial.println(c.getValue());
  
  d.reset();
  d.printDeck();
  d.shuffle();
  d.printDeck();
  */
  Card c('H', 2);
  Serial.print(c.getSuit());
  Serial.println(c.getValue());
}

void loop() {
  // put your main code here, to run repeatedly:

}
