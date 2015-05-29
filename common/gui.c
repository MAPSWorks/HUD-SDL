/*This source code copyrighted by Lazy Foo' Productions (2004-2015)
  and may not be redistributed without written permission.*/

//Using SDL, SDL_image, SDL_ttf, standard IO, math, and strings
#include "gui.h"

extern char sensors_buf[BUF_SIZE], bt_buf[BUF_SIZE];

//The window we'll be rendering to
SDL_Window* gWindow = NULL;

//The window renderer
SDL_Renderer* gRenderer = NULL;

//Globally used font
TTF_Font *gFont = NULL;

//Rendered texture
LTexture gTextTextureBT, gTextTextureSens;

//Current displayed the speedometer background, needle
LTexture gSpeedometerBackgroundTexture,gNeedleTexture;


bool init()
{
	//Initialization flag
	bool success = true;

	//Initialize SDL
	if( SDL_Init( SDL_INIT_VIDEO ) < 0 )
	{
		printf( "SDL could not initialize! SDL Error: %s\n", SDL_GetError() );
		success = false;
	}
	else
	{
		//Set texture filtering to linear
		if( !SDL_SetHint( SDL_HINT_RENDER_SCALE_QUALITY, "1" ) )
		{
			printf( "Warning: Linear texture filtering not enabled!" );
		}

		//Create window
		gWindow = SDL_CreateWindow( "HUD", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN );
		if( gWindow == NULL )
		{
			printf( "Window could not be created! SDL Error: %s\n", SDL_GetError() );
			success = false;
		}
		else
		{
			//Create vsynced renderer for window
			gRenderer = SDL_CreateRenderer( gWindow, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC );
			if( gRenderer == NULL )
			{
				printf( "Renderer could not be created! SDL Error: %s\n", SDL_GetError() );
				success = false;
			}
			else
			{
				//Initialize renderer color
				SDL_SetRenderDrawColor( gRenderer, 0, 0, 0, 0 );

				//Initialize PNG loading
				int imgFlags = IMG_INIT_PNG;
				if( !( IMG_Init( imgFlags ) & imgFlags ) )
				{
					printf( "SDL_image could not initialize! SDL_image Error: %s\n", IMG_GetError() );
					success = false;
				}

				//Initialize SDL_ttf
				if( TTF_Init() == -1 )
				{
					printf( "SDL_ttf could not initialize! SDL_ttf Error: %s\n", TTF_GetError() );
					success = false;
				}
			}
		}
	}

	return success;
}

bool loadMedia()
{
	//Loading success flag
	bool success = true;

	//Open the font
	gFont = TTF_OpenFont( "/usr/share/fonts/truetype/ubuntu-font-family/UbuntuMono-R.ttf", 28 );
	if( gFont == NULL )
	{
		printf( "Failed to load lazy font! SDL_ttf Error: %s\n", TTF_GetError() );
		success = false;
	}
	else
	{
		//Render text
		SDL_Color textColor = { 0xFF, 0xFF, 0xFF };
		if( !gTextTextureBT.loadFromRenderedText( "Tadaa", textColor,gFont,gRenderer ) || !gTextTextureSens.loadFromRenderedText( "Tadaa", textColor, gFont,gRenderer ))
		{
			printf( "Failed to render text texture!\n" );
			success = false;
		}

	}
	if( !gSpeedometerBackgroundTexture.loadFromFile( "/home/odroid/project/resources/step22.gif",gRenderer ) )
	{
		printf( "Failed to load speedometer backdround image - texture!\n" );
		success = false;
	}
	if( !gNeedleTexture.loadFromFile( "/home/odroid/project/resources/needle-fioptics.png",gRenderer ) )
	{
		printf( "Failed to load speedometer backdround image - texture!\n" );
		success = false;
	}

	return success;
}


bool reloadText()
{
	bool success = true;

	//Render text
	SDL_Color textColor = { 0xFF, 0xFF, 0xFF };

	if(!sensors_buf[0]) {
		if( !gTextTextureSens.loadFromRenderedText( "Sensors buffer empty", textColor, gFont,gRenderer) ) {
			printf( "Failed to render text texture!\n" );
			success = false;
		}
	} else {
		if( !gTextTextureSens.loadFromRenderedText( sensors_buf, textColor,gFont,gRenderer ) ) {
			printf( "Failed to render text texture!\n" );
			success = false;
		}
	}

	if(!bt_buf[0]) {
		if( !gTextTextureBT.loadFromRenderedText( "Bluetooth buffer empty", textColor,gFont,gRenderer ) ) {
			printf( "Failed to render text texture!\n" );
			success = false;
		}
	} else {
		if( !gTextTextureBT.loadFromRenderedText( bt_buf, textColor,gFont,gRenderer ) ) {
			printf( "Failed to render text texture!\n" );
			success = false;
		}
	}

	return success;
}

void close()
{
	//Free loaded images
	gTextTextureBT.free();
	gTextTextureSens.free();
	gSpeedometerBackgroundTexture.free();	
	gNeedleTexture.free();	

	//Free global font
	TTF_CloseFont( gFont );
	gFont = NULL;

	//Destroy window	
	SDL_DestroyRenderer( gRenderer );
	SDL_DestroyWindow( gWindow );
	gWindow = NULL;
	gRenderer = NULL;

	//Quit SDL subsystems
	TTF_Quit();
	IMG_Quit();
	SDL_Quit();
}

void* gui_main(void* arg)
{
	double degrees =0;
	//Start up SDL and create window
	if( !init() )
	{
		printf( "Failed to initialize!\n" );
	}
	else
	{
		//Load media
		if( !loadMedia() )
		{
			printf( "Failed to load media!\n" );
		}
		else
		{	
			//Main loop flag
			bool quit = false;

			//Event handler
			SDL_Event e;

			//While application is running
			while( !quit )
			{
				//Handle events on queue
				while( SDL_PollEvent( &e ) != 0 )
				{
					//User requests quit
					if( e.type == SDL_QUIT )
					{
						quit = true;
					}
				}
				degrees +=1;

				//Clear screen
				SDL_SetRenderDrawColor( gRenderer, 0, 0, 0, 0 );	//	background screen color
				SDL_RenderClear( gRenderer );
				

				//Render current frame
				gSpeedometerBackgroundTexture.render( ( SCREEN_WIDTH - gSpeedometerBackgroundTexture.getWidth() ) / 2, ( SCREEN_HEIGHT - gSpeedometerBackgroundTexture.getHeight() ) / 2,gRenderer, NULL, 0.0, NULL, SDL_FLIP_NONE );
				gNeedleTexture.render( ( SCREEN_WIDTH - gNeedleTexture.getWidth() ) / 2, ( SCREEN_HEIGHT - gNeedleTexture.getHeight() ) / 2,gRenderer, NULL, degrees, NULL, SDL_FLIP_NONE );
				reloadText();
				gTextTextureBT.render( ( SCREEN_WIDTH - gTextTextureBT.getWidth() ) / 2, ( SCREEN_HEIGHT - gTextTextureBT.getHeight() ) / 10,gRenderer );
				gTextTextureSens.render( ( SCREEN_WIDTH - gTextTextureSens.getWidth() ) / 2, ( SCREEN_HEIGHT - gTextTextureSens.getHeight() ) * 9 / 10,gRenderer );

				//Update screen
				SDL_RenderPresent( gRenderer );
			}
		}
	}

	//Free resources and close SDL
	close();

	return 0;
}
