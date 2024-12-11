import { createTheme } from '@mui/material/styles';

const clayTheme = createTheme({
  palette: {
    primary: {
      main: '#6CAF45', // Plant-like green
      light: '#98D87C',
      dark: '#478A2E',
      contrastText: '#ffffff',
    },
    secondary: {
      main: '#FFD700', // Golden yellow for accents
      light: '#FFEA70',
      dark: '#B29500',
      contrastText: '#000000',
    },
    background: {
      default: '#F4F4F9', // Light neutral for a clay-like surface
      paper: '#FFFFFF',
    },
    text: {
      primary: '#333333',
      secondary: '#666666',
    },
  },
  typography: {
    fontFamily: '"Roboto", "Arial", sans-serif',
    h1: {
      fontSize: '2.5rem',
      fontWeight: 700,
    },
    h2: {
      fontSize: '2rem',
      fontWeight: 600,
    },
    body1: {
      fontSize: '1rem',
      lineHeight: 1.6,
    },
    button: {
      textTransform: 'none',
      fontWeight: 600,
    },
  },
  shape: {
    borderRadius: 12, // Soft rounded corners
  },
  components: {
    MuiPaper: {
      styleOverrides: {
        root: {
          borderRadius: 16,
          boxShadow: '0px 4px 8px rgba(0, 0, 0, 0.1)', // Subtle shadows
        },
      },
    },
    MuiButton: {
      styleOverrides: {
        root: {
          borderRadius: 16,
          padding: '10px 20px',
          boxShadow: '0px 3px 6px rgba(0, 0, 0, 0.1)',
          ':hover': {
            boxShadow: '0px 6px 12px rgba(0, 0, 0, 0.15)', // Enhanced hover effect
          },
        },
      },
    },
  },
});

export default clayTheme;
