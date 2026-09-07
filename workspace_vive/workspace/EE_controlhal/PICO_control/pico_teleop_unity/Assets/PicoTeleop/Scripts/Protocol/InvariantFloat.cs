using System.Text;

namespace EgoARX.PicoTeleop.Protocol
{
    /// <summary>
    /// IL2CPP on PICO Android 10 aborts inside std::to_string(float) when C# calls
    /// float.ToString / interpolated :F2. Format through integers only.
    /// </summary>
    public static class InvariantFloat
    {
        public static void Append(StringBuilder sb, float value)
        {
            if (float.IsNaN(value) || float.IsInfinity(value))
            {
                sb.Append('0');
                return;
            }

            if (value < 0f)
            {
                sb.Append('-');
                value = -value;
            }

            const int decimals = 6;
            const int scale = 1000000;
            double scaled = (double)value * scale + 0.5;
            if (scaled >= int.MaxValue)
            {
                sb.Append("999999");
                return;
            }

            int total = (int)scaled;
            int integer = total / scale;
            int frac = total % scale;
            AppendNonNegativeInt(sb, integer);
            sb.Append('.');
            int pad = 100000;
            for (int i = 0; i < decimals; i++)
            {
                int digit = frac / pad;
                sb.Append((char)('0' + digit));
                frac -= digit * pad;
                pad /= 10;
            }
        }

        public static string Format(float value)
        {
            var sb = new StringBuilder(16);
            Append(sb, value);
            return sb.ToString();
        }

        static void AppendNonNegativeInt(StringBuilder sb, int value)
        {
            if (value == 0)
            {
                sb.Append('0');
                return;
            }

            char d0 = '0', d1 = '0', d2 = '0', d3 = '0', d4 = '0';
            char d5 = '0', d6 = '0', d7 = '0', d8 = '0', d9 = '0';
            int n = 0;
            while (value > 0)
            {
                char digit = (char)('0' + (value % 10));
                value /= 10;
                switch (n)
                {
                    case 0: d0 = digit; break;
                    case 1: d1 = digit; break;
                    case 2: d2 = digit; break;
                    case 3: d3 = digit; break;
                    case 4: d4 = digit; break;
                    case 5: d5 = digit; break;
                    case 6: d6 = digit; break;
                    case 7: d7 = digit; break;
                    case 8: d8 = digit; break;
                    default: d9 = digit; break;
                }

                n++;
            }

            for (int i = n - 1; i >= 0; i--)
            {
                switch (i)
                {
                    case 0: sb.Append(d0); break;
                    case 1: sb.Append(d1); break;
                    case 2: sb.Append(d2); break;
                    case 3: sb.Append(d3); break;
                    case 4: sb.Append(d4); break;
                    case 5: sb.Append(d5); break;
                    case 6: sb.Append(d6); break;
                    case 7: sb.Append(d7); break;
                    case 8: sb.Append(d8); break;
                    default: sb.Append(d9); break;
                }
            }
        }
    }
}
